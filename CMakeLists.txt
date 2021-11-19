cmake_minimum_required (VERSION 3.13)

PROJECT(bme680-mqtt)

set(
  file_h
    bme680.h
  )

set(
  file_c
    bme680.c
    main.c
  )

add_executable(
  ${PROJECT_NAME}
    ${file_h}
    ${file_c}
  )

target_link_libraries(
  ${PROJECT_NAME}
  paho-mqtt3c
  )
