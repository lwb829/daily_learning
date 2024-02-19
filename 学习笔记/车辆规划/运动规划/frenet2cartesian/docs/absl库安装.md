# install absl

## 安装
要在.h中 `#include <gtest/gtest.h>`，需要安装abseil库，安装步骤如下：

```bash
git clone https://github.com/abseil/abseil-cpp.git

cd abseil-cpp

mkdir build 

cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DCAMKE_CXX_STANDARD=11

make

sudo make install
```

**NOTE:**

- 使用C++标准11编译，则在工程中，也需要设置为C++标准11。11还是现在常用的版本，设置为17可能有点高（a mismatch in the C++ language standard used by different libraries）

- /usr/local is the default install path in ubuntu, you can use this command to find the absl install path:
  ```bash
  $ find /usr -name "absl"
  /usr/local/include/absl
  /usr/local/lib/cmake/absl
  ```

- to check the install path; if find it successfully, you can use `find_package(absl REQUIRED)` and link it to your project in CMakeLists.txt to use this third-party library.


## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10.2)
project(EMplanner)

find_package(absl REQUIRED)

include_directories(
  include
  ${absl_INCLUDE_DIRS}
)


add_library(
    PATH_SEARCHER_LIB SHARED
    ...
)

add_executable(test ...)
target_link_libraries(test PATH_SEARCHER_LIB absl::strings)
```