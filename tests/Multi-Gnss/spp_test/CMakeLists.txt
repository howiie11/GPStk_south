add_executable(r3Read r3Read.cpp)
target_link_libraries(r3Read pppbox)

add_executable(r3Read2 r3Read2.cpp)
target_link_libraries(r3Read2 pppbox)

add_executable(spp spp.cpp)
target_link_libraries(spp pppbox)
	
add_executable(satWatcher satWatcher.cpp)
target_link_libraries(satWatcher pppbox)


add_executable(whyTest whyTest.cpp)
target_link_libraries(whyTest pppbox)

