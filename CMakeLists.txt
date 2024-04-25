project(myusb)

add_executable(myusb usb.c usbstring.c)
add_compile_definitions(AIO)
target_link_libraries(myusb pthread aio)