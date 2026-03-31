# Fuzz targets will be added as domain code lands in Phase 2+
# Each fuzz target links liepp::liepp and uses -fsanitize=fuzzer

# Example pattern (uncomment when fuzz targets exist):
# add_executable(fuzz_so3_exp fuzz_so3_exp.cpp)
# target_link_libraries(fuzz_so3_exp PRIVATE liepp::liepp)
# target_compile_options(fuzz_so3_exp PRIVATE -fsanitize=fuzzer ${LIEPP_WARNING_FLAGS})
# target_link_options(fuzz_so3_exp PRIVATE -fsanitize=fuzzer)
