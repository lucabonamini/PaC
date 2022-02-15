option(PID "Enable compilation of Pid controller" ON)

# Pid
if(PID)
  message("Compiling Pid")
  add_subdirectory(pid)
endif()
