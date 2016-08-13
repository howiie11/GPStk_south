add_executable(ppp_test ppp_test.cpp)
target_link_libraries(ppp_test pppbox)

add_executable(sp3Read sp3Read.cpp)
target_link_libraries(sp3Read pppbox)
