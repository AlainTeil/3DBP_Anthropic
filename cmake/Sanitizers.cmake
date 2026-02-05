# Sanitizers.cmake
# Configures AddressSanitizer, UndefinedBehaviorSanitizer, and ThreadSanitizer

function(enable_sanitizers target_name)
    if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(SANITIZER_FLAGS "")
        
        if(BP3D_ENABLE_ASAN)
            list(APPEND SANITIZER_FLAGS "-fsanitize=address")
            list(APPEND SANITIZER_FLAGS "-fno-omit-frame-pointer")
            message(STATUS "AddressSanitizer enabled for ${target_name}")
        endif()
        
        if(BP3D_ENABLE_UBSAN)
            list(APPEND SANITIZER_FLAGS "-fsanitize=undefined")
            message(STATUS "UndefinedBehaviorSanitizer enabled for ${target_name}")
        endif()
        
        if(BP3D_ENABLE_TSAN)
            if(BP3D_ENABLE_ASAN)
                message(WARNING "ThreadSanitizer cannot be used with AddressSanitizer. Ignoring TSAN.")
            else()
                list(APPEND SANITIZER_FLAGS "-fsanitize=thread")
                message(STATUS "ThreadSanitizer enabled for ${target_name}")
            endif()
        endif()
        
        if(SANITIZER_FLAGS)
            target_compile_options(${target_name} PRIVATE ${SANITIZER_FLAGS})
            target_link_options(${target_name} PRIVATE ${SANITIZER_FLAGS})
        endif()
    else()
        message(WARNING "Sanitizers are only supported with GCC and Clang")
    endif()
endfunction()
