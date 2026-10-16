function(SCCopyRuntimeDependenciesToTarget destination_target source_target)
    if (NOT WIN32)
        return()
    endif()

    get_target_property(SC_RUNTIME_TARGET_TYPE ${source_target} TYPE)
    if(NOT SC_RUNTIME_TARGET_TYPE STREQUAL "SHARED_LIBRARY"
       AND NOT SC_RUNTIME_TARGET_TYPE STREQUAL "MODULE_LIBRARY")
        return()
    endif()

    add_custom_command(TARGET ${destination_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:${source_target}>
            $<TARGET_FILE_DIR:${destination_target}>
        VERBATIM
    )
endfunction()
