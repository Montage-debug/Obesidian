#json

https://github.com/nlohmann/json
该项目是单头文件库，下载后得到![[json.hpp]]

使用名字
```cpp
using json = nlohmann::json;
```

## ==创建JSON数据格式==
```cpp
json j;

// 添加键值对
j["name"] = "Alice";
j["age"] = 25;
j["is_student"] = true;

// 添加数组
j["hobbies"] = {"reading", "music", "sports"};

// 添加嵌套对象
j["address"] = {{"street", "123 Main St"}, {"city", "Wonderland"}};

// 序列化为字符串并输出
std::cout << j.dump(4) << std::endl; // 使用4个空格进行缩进
```


最后输出结果如下：
```json
{
    "address": {
        "city": "Wonderland",
        "street": "123 Main St"
    },
    "age": 25,
    "hobbies": [
        "reading",
        "music",
        "sports"
    ],
    "is_student": true,
    "name": "Alice"
}
```


## ==解析JSON格式==
`parse`方法解析JSON
```cpp
json::parse(JSON字符串)
```

比如从字符串解析JSON数据：
```cpp
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// 基本解析
std::string json_str = R"({"name": "Alice", "age": 25, "city": "New York"})";
json j = json::parse(json_str);

// 访问数据
std::string name = j["name"];
int age = j["age"];
std::string city = j["city"];
```


从文件解析：
```cpp
#include <fstream>
#include <sstream>

// 从文件读取JSON并解析
json parse_json_file(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    return json::parse(buffer.str());
}
```


复杂结构的JSON解析：
```cpp
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

int main() {
    std::string complex_json = R"({
        "users": [
            {
                "id": 1,
                "name": "Alice",
                "email": "alice@example.com",
                "preferences": {
                    "theme": "dark",
                    "notifications": true
                }
            },
            {
                "id": 2,
                "name": "Bob",
                "email": "bob@example.com",
                "preferences": {
                    "theme": "light",
                    "notifications": false
                }
            }
        ],
        "total_count": 2
    })";
    
    try {
        json data = json::parse(complex_json);
        
        // 遍历用户数组
        for (const auto& user : data["users"]) {
            std::cout << "ID: " << user["id"] 
                      << ", Name: " << user["name"] 
                      << ", Theme: " << user["preferences"]["theme"] 
                      << std::endl;
        }
        
        std::cout << "总用户数: " << data["total_count"] << std::endl;
        
    } catch (const json::parse_error& e) {
        std::cerr << "JSON解析错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```
