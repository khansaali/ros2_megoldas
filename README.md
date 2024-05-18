ros2_ws$ colcon build --packages-select my_cpp_pkg
Starting >>> my_cpp_pkg
--- stderr: my_cpp_pkg                             
/usr/bin/ld: /usr/lib/gcc/x86_64-linux-gnu/11/../../../x86_64-linux-gnu/Scrt1.o: in function `_start':
(.text+0x1b): undefined reference to `main'
collect2: error: ld returned 1 exit status
gmake[2]: *** [CMakeFiles/cpp_node.dir/build.make:151: cpp_node] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/cpp_node.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< my_cpp_pkg [0.89s, exited with code 2]

Summary: 0 packages finished [1.68s]
  1 package failed: my_cpp_pkg
  1 package had stderr output: my_cpp_pkg
