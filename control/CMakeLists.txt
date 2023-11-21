option(PID "Enable compilation of Pid controller" ON)
option(PURE_PURSUIT "Enable compilation of Pure Pursuit controller" ON)
option(DWA "Enable compilation of Dwa controller" ON)
option(MPC "Enable compilation of Mpc controller" OFF)

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

# Dwa
if(DWA)
  message("Compiling Dwa")
  add_subdirectory(dwa)
endif()

# Mpc
if(MPC)
  message("Compiling Mpc")
  add_subdirectory(mpc)
endif()
