add_subdirectory(MPU9250)

add_library(imu INTERFACE)
target_sources(imu INTERFACE ${CMAKE_CURRENT_LIST_DIR}/imu.cpp)

target_include_directories(imu INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(imu INTERFACE pico_stdlib MPU9250)

#add_library(mag INTERFACE)
#target_sources(mag INTERFACE ${CMAKE_CURRENT_LIST_DIR}/mag.cpp)

#target_include_directories(mag INTERFACE ${CMAKE_CURRENT_LIST_DIR})

#target_link_libraries(mag INTERFACE pico_stdlib MPU9250)


# 1)сначала задаем название библиотеки, под которым к ней и всем включенным 
# в нее функциям будут обращаться программы при ее (библиотеки) вызове
# 2)потом объявляем, на основе какого файла ( LCD.cpp ) из текущей директории 
# ( ${CMAKE_CURRENT_LIST_DIR} ) должна быть создана библиотека
# 3)далее объявляем, какие еще файлы включены в эту директорию и нужны для
# компилляции заданной цели-LCD (библиотеки)(как минимум - хэдеры(у нас так))
# 4)линкуем (устанавливаем взаимосвязи) библиотеку с другими библиотеками, 
# указанными как #include"..." (но не линкуем #include <...> - это системные  
# библиотеки, а также хэдеры) в LCD.cpp 
#
# заметим, что в самих файлах этой директории явно нет библиотек hardware/ и pico/,
# хотя они линкуются с проектом- наверное к ним реализуется обращение через 
# pico_sdk_import.cmake , а потому явно их объявлять, как и системные библиотеки, 
# не нужно. а так любые другие библиотеки нужно явно прописывать в директории
# и линковать, а указание на них реализуется (наверное) через 3 строку