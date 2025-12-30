# 04-代码库

本目录包含可复用的代码库、第三方库和算法实现。

---

## 📦 代码库分类

### 1. C++库
常用C++第三方库和工具类

#### httplib.h
- **功能**：轻量级HTTP客户端/服务器库
- **版本**：C++11单头文件
- **用途**：HTTP请求、RESTful API
- **文档**：[http库的使用](./C++库/http库的使用.md)

#### json.hpp
- **功能**：JSON解析库 (nlohmann/json)
- **版本**：C++11单头文件
- **用途**：JSON序列化/反序列化
- **文档**：[JSON库的使用](./C++库/JSON库的使用.md)

#### MD5
- **功能**：MD5哈希算法
- **用途**：数据校验、加密
- **文档**：[MD5的使用](./C++库/MD5的使用.md)

📁 [进入C++库目录](./C++库/)

---

### 2. 算法实现
自定义算法和实现方案

**包含内容：**
- 路径规划算法 (RRT, SC-RRT等)
- 控制算法
- 数据处理算法

📁 [进入算法实现目录](./算法实现/)

---

## 🔧 使用说明

### C++库集成方式

#### 1. httplib.h 使用示例
```cpp
#include "httplib.h"

// HTTP客户端
httplib::Client cli("http://localhost:8080");
auto res = cli.Get("/api/data");
if (res && res->status == 200) {
    std::cout << res->body << std::endl;
}

// HTTP服务器
httplib::Server svr;
svr.Get("/hello", [](const httplib::Request&, httplib::Response& res) {
    res.set_content("Hello World!", "text/plain");
});
svr.listen("localhost", 8080);
```

#### 2. json.hpp 使用示例
```cpp
#include "json.hpp"
using json = nlohmann::json;

// 解析JSON
json j = json::parse(R"({"name":"John","age":30})");
std::string name = j["name"];
int age = j["age"];

// 生成JSON
json obj;
obj["name"] = "Alice";
obj["age"] = 25;
std::string str = obj.dump();
```

#### 3. MD5 使用示例
```cpp
#include "md5.h"

std::string data = "Hello World";
std::string hash = MD5(data).toStr();
std::cout << "MD5: " << hash << std::endl;
```

---

## 📚 库依赖说明

### httplib.h
- **依赖**：C++11标准库
- **平台**：跨平台 (Windows, Linux, macOS)
- **编译选项**：需要链接pthread库 (Linux)

### json.hpp
- **依赖**：C++11标准库
- **平台**：跨平台
- **特点**：单头文件，无需编译

### MD5
- **依赖**：C++标准库
- **平台**：跨平台
- **性能**：适合小规模数据

---

## 🚀 最佳实践

### 1. 代码组织
```
project/
├── include/          # 头文件
│   ├── httplib.h
│   └── json.hpp
├── src/             # 源文件
├── lib/             # 静态/动态库
└── CMakeLists.txt   # 构建配置
```

### 2. CMake集成
```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(CMAKE_CXX_STANDARD 11)

# 添加头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include)

# 添加可执行文件
add_executable(myapp src/main.cpp)

# Linux下链接pthread
if(UNIX)
    target_link_libraries(myapp pthread)
endif()
```

### 3. 版本管理
- 使用Git子模块管理第三方库
- 记录库的版本号和来源
- 定期更新依赖库

---

## 📖 参考文档

### httplib
- [GitHub仓库](https://github.com/yhirose/cpp-httplib)
- [在线文档](https://github.com/yhirose/cpp-httplib/wiki)

### nlohmann/json
- [GitHub仓库](https://github.com/nlohmann/json)
- [在线文档](https://json.nlohmann.me/)

### MD5
- [MD5算法说明](https://en.wikipedia.org/wiki/MD5)

---

## 🔄 库更新记录

| 库名称 | 当前版本 | 最后更新 | 说明 |
|--------|---------|---------|------|
| httplib.h | - | 2025-12 | HTTP通信 |
| json.hpp | - | 2025-12 | JSON处理 |
| MD5 | - | 2025-12 | 哈希算法 |

---

## 💡 使用建议

1. **选择合适的库**：根据项目需求选择轻量级或功能完整的库
2. **性能考虑**：对性能敏感的场景，测试库的效率
3. **安全性**：注意库的安全更新，及时修复漏洞
4. **许可证**：确认第三方库的许可证与项目兼容

---

[返回主目录](../README.md)
