option(PID "Enable compilation of Pid controller" ON)
option(PURE_PURSUIT "Enable compilation of Pure Pursuit controller" ON)

# Pid
if(PID)
  message("Compiling Pid")
  add_subdirectory(pid)
endif()

# Pure Pursuit
if(PURE_PURSUIT)
  message("Compiling Pure Pursuit")
  add_subdirectory(pure_pursuit)
endif()

