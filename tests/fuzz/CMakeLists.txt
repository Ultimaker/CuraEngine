# By default we are going to use the libfuzzer engine. However if 
# LIB_FUZZING_ENGINE is declared you can override the fuzzing engine to one of;
# - Centipede
# - Hongfuzz
# - AFL++
# - etc.
set(LIB_FUZZING_ENGINE "$ENV{LIB_FUZZING_ENGINE}"
    CACHE STRING "Compiler flags necessary to link the fuzzing engine of choice e.g. libfuzzer, afl etc.")

set(FUZZ_TEST_SRC
        FuzzGcodeExport        
        )

foreach (test ${FUZZ_TEST_SRC})
    add_executable(${test} ${test}.cpp)
    target_link_libraries(${test} PRIVATE _CuraEngine clipper::clipper arcus::arcus test_helpers)
    # Optionally allow OSS-fuzz to manage engine flags directly.
    if (LIB_FUZZING_ENGINE)
      message(STATUS "Using custom fuzzing engine.")
      target_link_libraries(${test} PRIVATE "${LIB_FUZZING_ENGINE}")
    else ()
      # By default just build with address-sanitizers/libfuzzer for local testing
      message(STATUS "Using default fuzzing configuration libfuzzer+address-sanitizer.")
      target_compile_options(${test} PRIVATE "-fsanitize=fuzzer,address,undefined")
      target_link_libraries(${test} PRIVATE "-fsanitize=fuzzer,address,undefined")
    endif ()
endforeach ()
