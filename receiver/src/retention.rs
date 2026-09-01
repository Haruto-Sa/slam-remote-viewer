use std::{
    error::Error,
    fmt, fs, io,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
    time::{Duration, SystemTime},
};

use serde::Deserialize;

const CHECKPOINT_FILE: &str = "recording.inprogress.json";
const METADATA_FILE: &str = "metadata.json";

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RetentionPolicy {
    pub max_total_bytes: Option<u64>,
    pub max_age: Option<Duration>,
    pub dry_run: bool,
}

impl RetentionPolicy {
    pub fn is_enabled(self) -> bool {
        self.max_total_bytes.is_some() || self.max_age.is_some()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RetentionRemoval {
    pub directory: PathBuf,
    pub bytes: u64,
    pub reason: RetentionReason,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetentionReason {
    Age,
    Size,
}

impl fmt::Display for RetentionReason {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Age => formatter.write_str("age"),
            Self::Size => formatter.write_str("size"),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RetentionFailure {
    pub path: PathBuf,
    pub reason: String,
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct RetentionReport {
    pub total_bytes_before: u64,
    pub total_bytes_after: u64,
    pub removed: Vec<RetentionRemoval>,
    pub would_remove: Vec<RetentionRemoval>,
    pub failures: Vec<RetentionFailure>,
}

impl RetentionReport {
    pub fn reclaimed_bytes(&self) -> u64 {
        self.removed.iter().map(|removal| removal.bytes).sum()
    }

    pub fn potential_reclaimed_bytes(&self) -> u64 {
        self.would_remove.iter().map(|removal| removal.bytes).sum()
    }
}

#[derive(Clone)]
pub struct RetentionManager {
    root: PathBuf,
    policy: RetentionPolicy,
    lock: Arc<Mutex<()>>,
}

impl RetentionManager {
    pub fn new(root: impl Into<PathBuf>, policy: RetentionPolicy) -> Self {
        Self {
            root: root.into(),
            policy,
            lock: Arc::new(Mutex::new(())),
        }
    }

    pub fn policy(&self) -> RetentionPolicy {
        self.policy
    }

    pub fn apply(&self) -> RetentionReport {
        if !self.policy.is_enabled() {
            return RetentionReport::default();
        }
        let Ok(_guard) = self.lock.lock() else {
            return RetentionReport {
                failures: vec![RetentionFailure {
                    path: self.root.clone(),
                    reason: "retention lock was poisoned".to_owned(),
                }],
                ..RetentionReport::default()
            };
        };
        apply_retention_at(&self.root, self.policy, SystemTime::now())
    }

    pub fn apply_and_log(&self, trigger: &str) {
        if !self.policy.is_enabled() {
            return;
        }
        let report = self.apply();
        for removal in &report.removed {
            println!(
                "retention removed: trigger={trigger} reason={} bytes={} directory={}",
                removal.reason,
                removal.bytes,
                removal.directory.display()
            );
        }
        for removal in &report.would_remove {
            println!(
                "retention dry-run: trigger={trigger} reason={} bytes={} directory={}",
                removal.reason,
                removal.bytes,
                removal.directory.display()
            );
        }
        for failure in &report.failures {
            eprintln!(
                "retention failed: trigger={trigger} path={} reason={}",
                failure.path.display(),
                failure.reason
            );
        }
        println!(
            "retention completed: trigger={trigger} total_bytes_before={} total_bytes_after={} \
             reclaimed_bytes={} potential_reclaimed_bytes={} failures={}",
            report.total_bytes_before,
            report.total_bytes_after,
            report.reclaimed_bytes(),
            report.potential_reclaimed_bytes(),
            report.failures.len()
        );
    }
}

#[derive(Debug)]
struct Candidate {
    directory: PathBuf,
    canonical_directory: PathBuf,
    modified: SystemTime,
    bytes: u64,
    selected: bool,
}

#[derive(Deserialize)]
struct RetentionMetadata {
    protocol_version: u32,
    session: String,
    frame: String,
    unit: String,
    telemetry_file: String,
    pointcloud_file: String,
}

pub fn apply_retention(root: &Path, policy: RetentionPolicy) -> RetentionReport {
    apply_retention_at(root, policy, SystemTime::now())
}

fn apply_retention_at(root: &Path, policy: RetentionPolicy, now: SystemTime) -> RetentionReport {
    if !policy.is_enabled() || !root.exists() {
        return RetentionReport::default();
    }

    let mut report = RetentionReport::default();
    let canonical_root = match fs::canonicalize(root) {
        Ok(path) => path,
        Err(error) => {
            report.failures.push(failure(root, error));
            return report;
        }
    };
    let mut candidates = collect_candidates(root, &canonical_root, &mut report.failures);
    candidates.sort_by(|left, right| {
        left.modified
            .cmp(&right.modified)
            .then_with(|| left.directory.cmp(&right.directory))
    });

    report.total_bytes_before = candidates.iter().map(|candidate| candidate.bytes).sum();
    report.total_bytes_after = report.total_bytes_before;

    if let Some(max_age) = policy.max_age {
        for candidate in candidates.iter_mut().filter(|candidate| {
            now.duration_since(candidate.modified)
                .is_ok_and(|age| age > max_age)
        }) {
            select_candidate(
                candidate,
                RetentionReason::Age,
                &canonical_root,
                policy.dry_run,
                &mut report,
            );
        }
    }

    if let Some(max_total_bytes) = policy.max_total_bytes {
        for candidate in candidates
            .iter_mut()
            .filter(|candidate| !candidate.selected)
        {
            if report.total_bytes_after <= max_total_bytes {
                break;
            }
            select_candidate(
                candidate,
                RetentionReason::Size,
                &canonical_root,
                policy.dry_run,
                &mut report,
            );
        }
    }

    report
}

fn collect_candidates(
    root: &Path,
    canonical_root: &Path,
    failures: &mut Vec<RetentionFailure>,
) -> Vec<Candidate> {
    let mut candidates = Vec::new();
    let entries = match fs::read_dir(root) {
        Ok(entries) => entries,
        Err(error) => {
            failures.push(failure(root, error));
            return candidates;
        }
    };

    for entry in entries {
        let entry = match entry {
            Ok(entry) => entry,
            Err(error) => {
                failures.push(failure(root, error));
                continue;
            }
        };
        let path = entry.path();
        let file_type = match entry.file_type() {
            Ok(file_type) => file_type,
            Err(error) => {
                failures.push(failure(&path, error));
                continue;
            }
        };
        if entry.file_name() == "clips" {
            if file_type.is_symlink() {
                failures.push(retention_failure(
                    &path,
                    "clips directory must not be a symlink",
                ));
            } else if file_type.is_dir() {
                collect_container(&path, canonical_root, &mut candidates, failures);
            }
        } else if file_type.is_symlink() {
            failures.push(retention_failure(
                &path,
                "recording directory must not be a symlink",
            ));
        } else if file_type.is_dir()
            && let Some(candidate) = inspect_candidate(&path, canonical_root, failures)
        {
            candidates.push(candidate);
        }
    }
    candidates
}

fn collect_container(
    container: &Path,
    canonical_root: &Path,
    candidates: &mut Vec<Candidate>,
    failures: &mut Vec<RetentionFailure>,
) {
    let entries = match fs::read_dir(container) {
        Ok(entries) => entries,
        Err(error) => {
            failures.push(failure(container, error));
            return;
        }
    };
    for entry in entries {
        let entry = match entry {
            Ok(entry) => entry,
            Err(error) => {
                failures.push(failure(container, error));
                continue;
            }
        };
        let path = entry.path();
        match entry.file_type() {
            Ok(file_type) if file_type.is_dir() => {
                if let Some(candidate) = inspect_candidate(&path, canonical_root, failures) {
                    candidates.push(candidate);
                }
            }
            Ok(file_type) if file_type.is_symlink() => failures.push(retention_failure(
                &path,
                "clip directory must not be a symlink",
            )),
            Ok(_) => {}
            Err(error) => failures.push(failure(&path, error)),
        }
    }
}

fn inspect_candidate(
    directory: &Path,
    canonical_root: &Path,
    failures: &mut Vec<RetentionFailure>,
) -> Option<Candidate> {
    if directory.join(CHECKPOINT_FILE).exists() {
        return None;
    }
    let canonical_directory = match fs::canonicalize(directory) {
        Ok(path) if path.starts_with(canonical_root) && path != canonical_root => path,
        Ok(_) => {
            failures.push(retention_failure(
                directory,
                "recording resolves outside the configured root",
            ));
            return None;
        }
        Err(error) => {
            failures.push(failure(directory, error));
            return None;
        }
    };

    let metadata_path = directory.join(METADATA_FILE);
    let metadata_fs = match fs::symlink_metadata(&metadata_path) {
        Ok(metadata) if metadata.file_type().is_symlink() => {
            failures.push(retention_failure(
                &metadata_path,
                "metadata must not be a symlink",
            ));
            return None;
        }
        Ok(metadata) if metadata.is_file() => metadata,
        Ok(_) => {
            failures.push(retention_failure(
                &metadata_path,
                "metadata is not a regular file",
            ));
            return None;
        }
        Err(error) => {
            failures.push(failure(&metadata_path, error));
            return None;
        }
    };
    let metadata: RetentionMetadata = match FileReader::read_json(&metadata_path) {
        Ok(metadata) => metadata,
        Err(reason) => {
            failures.push(retention_failure(&metadata_path, reason));
            return None;
        }
    };
    if let Err(reason) = validate_metadata(&metadata) {
        failures.push(retention_failure(&metadata_path, reason));
        return None;
    }
    if let Err(reason) = validate_recording_files(directory, &metadata) {
        failures.push(retention_failure(directory, reason));
        return None;
    }

    let bytes = match directory_size(directory) {
        Ok(bytes) => bytes,
        Err(error) => {
            failures.push(failure(directory, error));
            return None;
        }
    };
    let modified = match metadata_fs.modified() {
        Ok(modified) => modified,
        Err(error) => {
            failures.push(failure(&metadata_path, error));
            return None;
        }
    };
    Some(Candidate {
        directory: directory.to_owned(),
        canonical_directory,
        modified,
        bytes,
        selected: false,
    })
}

fn select_candidate(
    candidate: &mut Candidate,
    reason: RetentionReason,
    canonical_root: &Path,
    dry_run: bool,
    report: &mut RetentionReport,
) {
    let removal = RetentionRemoval {
        directory: candidate.directory.clone(),
        bytes: candidate.bytes,
        reason,
    };
    if dry_run {
        candidate.selected = true;
        report.total_bytes_after = report.total_bytes_after.saturating_sub(candidate.bytes);
        report.would_remove.push(removal);
        return;
    }

    candidate.selected = true;
    match remove_candidate(candidate, canonical_root) {
        Ok(()) => {
            report.total_bytes_after = report.total_bytes_after.saturating_sub(candidate.bytes);
            report.removed.push(removal);
        }
        Err(reason) => report
            .failures
            .push(retention_failure(&candidate.directory, reason)),
    }
}

fn remove_candidate(candidate: &Candidate, canonical_root: &Path) -> Result<(), String> {
    if candidate.directory.join(CHECKPOINT_FILE).exists() {
        return Err("recording became active or incomplete before removal".to_owned());
    }
    let metadata = fs::symlink_metadata(&candidate.directory).map_err(|error| error.to_string())?;
    if !metadata.is_dir() || metadata.file_type().is_symlink() {
        return Err("recording directory is no longer a regular directory".to_owned());
    }
    let canonical = fs::canonicalize(&candidate.directory).map_err(|error| error.to_string())?;
    if canonical != candidate.canonical_directory || !canonical.starts_with(canonical_root) {
        return Err("recording path changed or resolves outside the configured root".to_owned());
    }
    let metadata_path = candidate.directory.join(METADATA_FILE);
    let metadata_fs = fs::symlink_metadata(&metadata_path).map_err(|error| error.to_string())?;
    if !metadata_fs.is_file() || metadata_fs.file_type().is_symlink() {
        return Err("recording metadata changed before removal".to_owned());
    }
    let metadata: RetentionMetadata = FileReader::read_json(&metadata_path)?;
    validate_metadata(&metadata)?;
    validate_recording_files(&candidate.directory, &metadata)?;
    fs::remove_dir_all(&candidate.directory).map_err(|error| error.to_string())
}

fn validate_metadata(metadata: &RetentionMetadata) -> Result<(), String> {
    if metadata.protocol_version != 1 {
        return Err(format!(
            "protocol_version must be 1, received {}",
            metadata.protocol_version
        ));
    }
    if metadata.session.trim().is_empty() {
        return Err("session must not be empty".to_owned());
    }
    if metadata.frame != "unity_world" || metadata.unit != "m" {
        return Err("metadata must use unity_world coordinates in metres".to_owned());
    }
    validate_filename("telemetry_file", &metadata.telemetry_file)?;
    validate_filename("pointcloud_file", &metadata.pointcloud_file)
}

fn validate_filename(field: &str, filename: &str) -> Result<(), String> {
    if filename.is_empty()
        || Path::new(filename)
            .file_name()
            .and_then(|name| name.to_str())
            != Some(filename)
    {
        return Err(format!("{field} must be a plain filename"));
    }
    Ok(())
}

fn validate_recording_files(directory: &Path, metadata: &RetentionMetadata) -> Result<(), String> {
    for (field, filename) in [
        ("telemetry_file", metadata.telemetry_file.as_str()),
        ("pointcloud_file", metadata.pointcloud_file.as_str()),
    ] {
        let path = directory.join(filename);
        let file_metadata = fs::symlink_metadata(&path)
            .map_err(|error| format!("failed to inspect {field} {}: {error}", path.display()))?;
        if !file_metadata.is_file() || file_metadata.file_type().is_symlink() {
            return Err(format!("{field} must reference a regular file"));
        }
    }
    Ok(())
}

fn directory_size(path: &Path) -> io::Result<u64> {
    let mut total = 0_u64;
    let mut pending = vec![path.to_owned()];
    while let Some(directory) = pending.pop() {
        for entry in fs::read_dir(directory)? {
            let entry = entry?;
            let metadata = fs::symlink_metadata(entry.path())?;
            if metadata.is_dir() && !metadata.file_type().is_symlink() {
                pending.push(entry.path());
            } else {
                total = total.saturating_add(metadata.len());
            }
        }
    }
    Ok(total)
}

struct FileReader;

impl FileReader {
    fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T, String> {
        let file = fs::File::open(path).map_err(|error| error.to_string())?;
        serde_json::from_reader(file).map_err(|error| error.to_string())
    }
}

fn failure(path: &Path, error: io::Error) -> RetentionFailure {
    retention_failure(path, error.to_string())
}

fn retention_failure(path: &Path, reason: impl Into<String>) -> RetentionFailure {
    RetentionFailure {
        path: path.to_owned(),
        reason: reason.into(),
    }
}

impl fmt::Debug for RetentionManager {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("RetentionManager")
            .field("root", &self.root)
            .field("policy", &self.policy)
            .finish_non_exhaustive()
    }
}

impl Error for RetentionFailure {}

impl fmt::Display for RetentionFailure {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}: {}", self.path.display(), self.reason)
    }
}

#[cfg(test)]
mod tests {
    use std::{
        fs::{self, File, FileTimes},
        sync::atomic::{AtomicU64, Ordering},
    };

    use super::*;

    static NEXT_TEST_DIRECTORY: AtomicU64 = AtomicU64::new(0);

    struct TestDirectory(PathBuf);

    impl TestDirectory {
        fn new(name: &str) -> Self {
            let id = NEXT_TEST_DIRECTORY.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "slam-receiver-retention-{}-{name}-{id}",
                std::process::id()
            ));
            if path.exists() {
                fs::remove_dir_all(&path).expect("stale test directory should be removable");
            }
            fs::create_dir_all(&path).expect("test root should be created");
            Self(path)
        }
    }

    impl Drop for TestDirectory {
        fn drop(&mut self) {
            if self.0.exists() {
                fs::remove_dir_all(&self.0).expect("test directory should be removable");
            }
        }
    }

    fn finalized_recording(
        directory: &Path,
        session: &str,
        payload_bytes: usize,
        modified: SystemTime,
    ) -> u64 {
        fs::create_dir_all(directory).expect("recording directory should be created");
        fs::write(
            directory.join("telemetry.ndjson"),
            vec![b'x'; payload_bytes],
        )
        .expect("telemetry should be written");
        fs::write(directory.join("pointcloud.ply"), b"ply\n")
            .expect("pointcloud should be written");
        let metadata_path = directory.join(METADATA_FILE);
        fs::write(
            &metadata_path,
            format!(
                r#"{{"protocol_version":1,"session":"{session}","frame":"unity_world","unit":"m","telemetry_file":"telemetry.ndjson","pointcloud_file":"pointcloud.ply"}}"#
            ),
        )
        .expect("metadata should be written");
        File::options()
            .write(true)
            .open(metadata_path)
            .expect("metadata should open")
            .set_times(FileTimes::new().set_modified(modified))
            .expect("metadata timestamp should be set");
        directory_size(directory).expect("recording size should be readable")
    }

    #[test]
    fn retention_is_disabled_without_limits() {
        let root = TestDirectory::new("disabled");
        let recording = root.0.join("recording");
        finalized_recording(&recording, "disabled", 32, SystemTime::UNIX_EPOCH);

        let report = apply_retention(&root.0, RetentionPolicy::default());

        assert!(recording.exists());
        assert_eq!(report, RetentionReport::default());
    }

    #[test]
    fn size_limit_removes_oldest_finalized_recording_first() {
        let root = TestDirectory::new("size-order");
        let now = SystemTime::now();
        let oldest = root.0.join("a-oldest");
        let newest = root.0.join("b-newest");
        finalized_recording(&oldest, "same-size", 64, now - Duration::from_secs(20));
        let newest_size =
            finalized_recording(&newest, "same-size", 64, now - Duration::from_secs(10));

        let report = apply_retention_at(
            &root.0,
            RetentionPolicy {
                max_total_bytes: Some(newest_size),
                ..RetentionPolicy::default()
            },
            now,
        );

        assert_eq!(report.removed.len(), 1);
        assert_eq!(report.removed[0].directory, oldest);
        assert_eq!(report.removed[0].reason, RetentionReason::Size);
        assert!(!oldest.exists());
        assert!(newest.exists());
        assert!(report.failures.is_empty());
    }

    #[test]
    fn equal_timestamps_are_ordered_by_directory_path() {
        let root = TestDirectory::new("path-order");
        let modified = SystemTime::now();
        let first = root.0.join("a-recording");
        let second = root.0.join("b-recording");
        finalized_recording(&first, "same-size", 64, modified);
        let second_size = finalized_recording(&second, "same-size", 64, modified);

        let report = apply_retention_at(
            &root.0,
            RetentionPolicy {
                max_total_bytes: Some(second_size),
                ..RetentionPolicy::default()
            },
            modified,
        );

        assert_eq!(report.removed.len(), 1);
        assert_eq!(report.removed[0].directory, first);
        assert!(!first.exists());
        assert!(second.exists());
    }

    #[test]
    fn age_limit_applies_to_full_sessions_and_clips() {
        let root = TestDirectory::new("age");
        let now = SystemTime::now();
        let old_session = root.0.join("old-session");
        let old_clip = root.0.join("clips").join("old-clip");
        let recent = root.0.join("recent-session");
        finalized_recording(
            &old_session,
            "old-session",
            8,
            now - Duration::from_secs(3 * 24 * 60 * 60),
        );
        finalized_recording(
            &old_clip,
            "old-clip",
            8,
            now - Duration::from_secs(4 * 24 * 60 * 60),
        );
        finalized_recording(&recent, "recent", 8, now - Duration::from_secs(60));

        let report = apply_retention_at(
            &root.0,
            RetentionPolicy {
                max_age: Some(Duration::from_secs(2 * 24 * 60 * 60)),
                ..RetentionPolicy::default()
            },
            now,
        );

        assert_eq!(report.removed.len(), 2);
        assert!(
            report
                .removed
                .iter()
                .all(|item| item.reason == RetentionReason::Age)
        );
        assert!(!old_session.exists());
        assert!(!old_clip.exists());
        assert!(recent.exists());
        assert!(report.failures.is_empty());
    }

    #[test]
    fn dry_run_reports_candidates_without_removing_them() {
        let root = TestDirectory::new("dry-run");
        let recording = root.0.join("recording");
        finalized_recording(&recording, "dry-run", 32, SystemTime::UNIX_EPOCH);

        let report = apply_retention_at(
            &root.0,
            RetentionPolicy {
                max_age: Some(Duration::from_secs(1)),
                dry_run: true,
                ..RetentionPolicy::default()
            },
            SystemTime::UNIX_EPOCH + Duration::from_secs(10),
        );

        assert!(recording.exists());
        assert!(report.removed.is_empty());
        assert_eq!(report.would_remove.len(), 1);
        assert_eq!(report.total_bytes_after, 0);
    }

    #[test]
    fn protects_in_progress_and_malformed_recordings_while_continuing() {
        let root = TestDirectory::new("protected");
        let now = SystemTime::now();
        let active = root.0.join("active");
        finalized_recording(&active, "active", 16, now);
        fs::write(active.join(CHECKPOINT_FILE), b"in progress")
            .expect("checkpoint should be written");

        let malformed = root.0.join("malformed");
        fs::create_dir_all(&malformed).expect("malformed directory should be created");
        fs::write(malformed.join(METADATA_FILE), b"not JSON")
            .expect("malformed metadata should be written");

        let eligible = root.0.join("eligible");
        finalized_recording(&eligible, "eligible", 16, now);

        let report = apply_retention_at(
            &root.0,
            RetentionPolicy {
                max_total_bytes: Some(1),
                ..RetentionPolicy::default()
            },
            now,
        );

        assert!(active.exists());
        assert!(malformed.exists());
        assert!(!eligible.exists());
        assert_eq!(report.removed.len(), 1);
        assert_eq!(report.failures.len(), 1);
        assert_eq!(report.failures[0].path, malformed.join(METADATA_FILE));
    }

    #[cfg(unix)]
    #[test]
    fn rejects_symlinked_recording_directories_without_touching_targets() {
        use std::os::unix::fs::symlink;

        let root = TestDirectory::new("symlink-root");
        let outside = TestDirectory::new("symlink-target");
        let target = outside.0.join("target");
        finalized_recording(&target, "outside", 16, SystemTime::UNIX_EPOCH);
        let link = root.0.join("linked-recording");
        symlink(&target, &link).expect("recording symlink should be created");

        let report = apply_retention_at(
            &root.0,
            RetentionPolicy {
                max_total_bytes: Some(1),
                ..RetentionPolicy::default()
            },
            SystemTime::now(),
        );

        assert!(target.exists());
        assert!(link.exists());
        assert!(report.removed.is_empty());
        assert_eq!(report.failures.len(), 1);
        assert_eq!(report.failures[0].path, link);
    }
}
