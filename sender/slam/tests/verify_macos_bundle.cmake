if(NOT EXISTS "${PLIST}")
    message(FATAL_ERROR "bundle Info.plist does not exist: ${PLIST}")
endif()
if(NOT EXISTS "${EXECUTABLE}")
    message(FATAL_ERROR "bundle executable does not exist: ${EXECUTABLE}")
endif()

execute_process(
    COMMAND /usr/bin/plutil -lint "${PLIST}"
    RESULT_VARIABLE lint_result
    OUTPUT_VARIABLE lint_output
    ERROR_VARIABLE lint_error
)
if(NOT lint_result EQUAL 0)
    message(FATAL_ERROR "invalid bundle plist: ${lint_output}${lint_error}")
endif()

file(READ "${PLIST}" plist_contents)
foreach(required
        "<string>SLAM Live Sender</string>"
        "<string>io.github.haruto-sa.slam-remote-viewer.sender</string>"
        "<string>Capture camera frames for local SLAM processing and diagnostics.</string>"
        "<string>0.1.0</string>"
        "<string>12.0</string>"
        "<string>APPL</string>")
    string(FIND "${plist_contents}" "${required}" required_position)
    if(required_position EQUAL -1)
        message(FATAL_ERROR "bundle plist is missing ${required}")
    endif()
endforeach()

string(FIND "${plist_contents}" "<key>LSMultipleInstancesProhibited</key>" single_instance_position)
if(single_instance_position EQUAL -1)
    message(FATAL_ERROR "bundle plist must prohibit multiple LaunchServices instances")
endif()
