# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

#set(COMPONENT_REQUIRES "bb_spi_lcd")
set(COMPONENT_REQUIRES "tmc")
idf_component_register(
    SRCS "main.cpp" 
    "bb_spi_lcd/bb_spi_lcd.cpp" "bb_spi_lcd/bb_parallel.cpp"
    "SDManager/SDManager.cpp" "Stepper/StepperMotor.cpp" "servo/servoControl.cpp"
      # list the source files of this component
    INCLUDE_DIRS "."      # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES            # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
)

