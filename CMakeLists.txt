project(Nucleo_Rosebot_David C CXX ASM)
cmake_minimum_required( VERSION 2.8.4)


include(../../mbed.gcc-arm.nucluo-f401re.cmake)


#enable_language(ASM)

INCLUDE_DIRECTORIES("BufferedSerial")
INCLUDE_DIRECTORIES("Buffer")
INCLUDE_DIRECTORIES("QEI")
INCLUDE_DIRECTORIES("MotorCtrl")
INCLUDE_DIRECTORIES("MotorDriver")
INCLUDE_DIRECTORIES("pid")
INCLUDE_DIRECTORIES("Trajectory")
INCLUDE_DIRECTORIES("SerialCom")
INCLUDE_DIRECTORIES(".")



#LINK_DIRECTORIES("Libs")


## Add all sources
file(GLOB_RECURSE SRC_FILES "*.cpp")
file(GLOB_RECURSE INC_FILES "*.h")

add_executable(${PROJECT_NAME} ${SRC_FILES} ${INC_FILES})

make_mbed_firmware(${PROJECT_NAME})


#install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION .)
