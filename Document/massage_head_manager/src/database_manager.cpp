#include "massage_head_manager/database_manager.hpp"         
#include "massage_head_manager/protocol_utils.hpp"
#include <nlohmann/json.hpp>
#include <mutex>
#include <map>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// 数据库密码常量
static constexpr const char* DB_PASSWORD = "wlzc@M!2b%tx";

using json = nlohmann::json;    
namespace massage_head_manager                               
 {                                                            
                                                              
 DatabaseManager::DatabaseManager(const rclcpp::Logger& logger) : logger_(logger), db_(nullptr), is_initialized_(false)  
 {                                                            
 }            
 
 
 DatabaseManager::~DatabaseManager()
 {
    close();
 }


 bool DatabaseManager::initialize(const std::string& db_path) 
 {
     // 从环境变量获取数据库路径
     std::string actual_db_path = db_path;
     if (actual_db_path.empty()) 
     {
         const char* env_path = std::getenv("MR_DATABASE_PATH");
         if (env_path != nullptr && std::strlen(env_path) > 0) 
         {
             actual_db_path = env_path;
         } 
     }
     
     // 打开数据库连接
     int result = sqlite3_open(actual_db_path.c_str(), &db_);
     if (result != SQLITE_OK) {
         RCLCPP_ERROR(logger_, "打开数据库失败: %s", sqlite3_errmsg(db_));
         return false;
     }

     // 设置数据库密钥
     std::string key_sql = std::string("PRAGMA key = '") + DB_PASSWORD + "';";
     char* err_msg = nullptr;
     result = sqlite3_exec(db_, key_sql.c_str(), nullptr, nullptr, &err_msg);
     if (result != SQLITE_OK)
     {
         RCLCPP_ERROR(logger_, "设置数据库密钥失败: %s", err_msg ? err_msg : "Unknown error");
         if (err_msg) sqlite3_free(err_msg);
         sqlite3_close(db_);
         db_ = nullptr;
         return false;
     }
     
     // 设置SQLCipher兼容性
     sqlite3_exec(db_, "PRAGMA cipher_compatibility = 3;", nullptr, nullptr, nullptr);
     
     RCLCPP_INFO(logger_, "数据库连接成功: %s", actual_db_path.c_str());
                                                 
    is_initialized_ = true;
    db_path_ = actual_db_path;

    // 全量加载 massage_head 行并解析 tcp，构建缓存索引
    {
        std::lock_guard<std::mutex> lock(db_mutex_);
        cached_heads_ = getAllMassageHead();
        
        cached_by_serial_.clear();
        cached_head_tcps_.clear();
    }

    for (const auto& head : cached_heads_) {
        RCLCPP_INFO(logger_, "加载按摩头: name=%s, id=%s, serial=%s", 
                   head.name.c_str(), head.id.c_str(), head.serial_number.c_str()); 

        // 构建 serial_number 索引
        if (!head.serial_number.empty()) {
            cached_by_serial_[head.serial_number] = head;
        }
        

        // 解析并缓存 TCP 数据
        if (!head.config_data.empty()) {
            auto tcp_map = parseTcpFromConfigData(head.config_data);
            if (!head.serial_number.empty()) {
                cached_head_tcps_[head.serial_number] = tcp_map;
            }

            RCLCPP_INFO(logger_, "  解析到 %zu 个机器人的TCP坐标", tcp_map.size());
            for (const auto& [robot_name, tcp] : tcp_map) {
                RCLCPP_INFO(logger_, "    [%s]: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f",
                           robot_name.c_str(), tcp.x, tcp.y, tcp.z, tcp.rx, tcp.ry, tcp.rz);
            }
        }
    }

    RCLCPP_INFO(logger_, "数据库初始化完成: heads=%zu, serial_index=%zu, tcp_cache=%zu",
               cached_heads_.size(), cached_by_serial_.size(),
               cached_head_tcps_.size());
    return true;
                                            
 }                                                            

// 从缓存中获取按摩头信息（通过 serial_number）
MassageHead DatabaseManager::getMassageHeadFromCache(const std::string& serial_number)
{
    std::lock_guard<std::mutex> lock(db_mutex_);
    auto it = cached_by_serial_.find(serial_number);
    if (it != cached_by_serial_.end()) {
        RCLCPP_DEBUG(logger_, "从缓存获取按摩头: serial_number=%s", serial_number.c_str());
        return it->second;
    }
    
    RCLCPP_WARN(logger_, "缓存中未找到按摩头: serial_number=%s", serial_number.c_str());
    return MassageHead();
}

// 从缓存中获取TCP数据（通过 serial_number）
std::map<std::string, TcpPosition> DatabaseManager::getTcpFromCache(const std::string& serial_number)
{
    std::lock_guard<std::mutex> lock(db_mutex_);
    auto it = cached_head_tcps_.find(serial_number);
    if (it != cached_head_tcps_.end()) {
        RCLCPP_DEBUG(logger_, "从缓存获取TCP数据: serial_number=%s -> %zu个机器人", 
                    serial_number.c_str(), it->second.size());
        return it->second;
    }
    
    RCLCPP_WARN(logger_, "缓存中未找到TCP数据: serial_number=%s", serial_number.c_str());
    return std::map<std::string, TcpPosition>();
}

  // 保存按摩头配置
  bool DatabaseManager::saveMassageHead(const MassageHead& 
  config)
  {
      if (!is_initialized_ || !db_) {
          RCLCPP_ERROR(logger_, "数据库未初始化");
          return false;
      }

      const std::string sql = R"(
          INSERT OR REPLACE INTO massage_head 
          (name, description, updated_at)
          VALUES (?, ?, datetime('now'))
      )";

      sqlite3_stmt* stmt = nullptr;
      int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);

      if (result != SQLITE_OK) {
          RCLCPP_ERROR(logger_, "准备配置SQL语句失败: %s", sqlite3_errmsg(db_));
          return false;
      }

      sqlite3_bind_text(stmt, 1, config.name.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 2, config.description.c_str(), -1, SQLITE_TRANSIENT);

      result = sqlite3_step(stmt);
      sqlite3_finalize(stmt);

      if (result != SQLITE_DONE) {
          RCLCPP_ERROR(logger_, "执行配置SQL失败: %s", sqlite3_errmsg(db_));
          return false;
      }

      RCLCPP_INFO(logger_, "按摩头配置保存成功: name=%s", config.name.c_str());
      return true;
  }


  // 获取所有按摩头配置
  std::vector<MassageHead> DatabaseManager::getAllMassageHead()
  {
    std::vector<MassageHead> configs;

    if (!is_initialized_ || !db_) {
        RCLCPP_ERROR(logger_, "数据库未初始化");
        return configs;
    }

    const std::string sql = R"(
    SELECT id, serial_number, name, description, image_url, icon_url, config_data, switch_command, created_at, updated_at
    FROM massage_head)";
    sqlite3_stmt* stmt = nullptr;
    int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);

    if (result != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "准备查询所有配置SQL语句失败: %s", sqlite3_errmsg(db_));
        return configs;
    }

    // 循环读取所有记录
    while ((result = sqlite3_step(stmt)) == SQLITE_ROW) 
    {
        MassageHead config;
        const unsigned char* txt;
        txt = sqlite3_column_text(stmt, 0);
        config.id = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 1);
        std::string raw_serial = txt ? reinterpret_cast<const char*>(txt) : "";
        config.serial_number = massage_head_manager::utils::hexHyphensToSpaces(raw_serial);

        txt = sqlite3_column_text(stmt, 2);
        config.name = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 3);
        config.description = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 4);
        config.image_url = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 5);
        config.icon_url = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 6);
        config.config_data = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 7);
        config.switch_command = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 8);
        config.created_at = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 9);
        config.updated_at = txt ? reinterpret_cast<const char*>(txt) : "";

        configs.push_back(config);
    }

    sqlite3_finalize(stmt);

    if (configs.empty()) {
        RCLCPP_INFO(logger_, "数据库中没有找到按摩头配置");
    } else {
        RCLCPP_INFO(logger_, "获取到 %zu 个按摩头配置", configs.size());
    }

    return configs;

  }

  // 解析 config_data 字段，返回 map: robot_name -> {x,y,z}
std::map<std::string, massage_head_manager::TcpPosition>
DatabaseManager::parseTcpFromConfigData(const std::string& config_json)
{
    std::map<std::string, massage_head_manager::TcpPosition> tcp_map;
    try {
        auto json_data = nlohmann::json::parse(config_json);

        if (json_data.contains("tcp")) {
            for (auto& [key, value] : json_data["tcp"].items()) {
                massage_head_manager::TcpPosition tcp;
                tcp.x = value.value("x", 0.0);
                tcp.y = value.value("y", 0.0);
                tcp.z = value.value("z", 0.0);
                tcp.rx = value.value("rx", 0.0);
                tcp.ry = value.value("ry", 0.0);
                tcp.rz = value.value("rz", 0.0);
                tcp_map[key] = tcp;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("DatabaseManager"), "Failed to parse TCP from config: %s", e.what());
    }
    return tcp_map;
}

// 解析 config_data 字段，返回 payload mass 值
double DatabaseManager::parsePayloadFromConfigData(const std::string& config_json)
{
    double mass = 0.0;
    try {
        auto json_data = nlohmann::json::parse(config_json);

        if (json_data.contains("payload") && json_data["payload"].contains("mass")) {
            mass = json_data["payload"]["mass"].get<double>();
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("DatabaseManager"), "Failed to parse payload from config: %s", e.what());
    }
    return mass;
}



  // 关闭数据库连接
  void DatabaseManager::close()
  {
      if (db_) {
          sqlite3_close(db_);
          db_ = nullptr;
      }
      is_initialized_ = false;
  }




std::map<std::string, TcpPosition>
  DatabaseManager::getMassageHeadTcpBySerialNumber(const std::string&
  serial_number)
  {
      std::map<std::string, TcpPosition> tcp_map;

      if (!is_initialized_ || !db_) {
          RCLCPP_ERROR(logger_, "数据库未初始化");
          return tcp_map;
      }

      // serial_number作为key查缓存
      {
          std::lock_guard<std::mutex> lock(db_mutex_);
          auto it = cached_head_tcps_.find(serial_number);  // 直接用serial_number查找
          if (it != cached_head_tcps_.end()) 
          {
              return it->second;
          }
      }

      // 转换serial_number格式：空格 -> 连字符（数据库存储格式）
      std::string db_serial_number = massage_head_manager::utils::hexSpacesToHyphens(serial_number);
      RCLCPP_DEBUG(logger_, "查询TCP数据: serial_number=%s -> db格式=%s", 
                  serial_number.c_str(), db_serial_number.c_str());

      // 第二步：查询数据库并解析
      const std::string sql = "SELECT config_data FROM massage_head WHERE serial_number = ?";
      sqlite3_stmt* stmt = nullptr;
      int res = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
      if (res != SQLITE_OK) {
          RCLCPP_ERROR(logger_, "准备查询config_data SQL失败: %s",
  sqlite3_errmsg(db_));
          return tcp_map;
      }

      sqlite3_bind_text(stmt, 1, db_serial_number.c_str(), -1,
  SQLITE_TRANSIENT);
      res = sqlite3_step(stmt);

      if (res == SQLITE_ROW) {
          const char* txt = reinterpret_cast<const
  char*>(sqlite3_column_text(stmt, 0));
          if (txt) {
              std::string json_str = txt;
              tcp_map = parseTcpFromConfigData(json_str);

              // 第三步：直接用serial_number作为key缓存
              {
                  std::lock_guard<std::mutex> lock(db_mutex_);
                  cached_head_tcps_[serial_number] = tcp_map;
              }

              RCLCPP_INFO(logger_, "数据库查询TCP数据: serial_number=%s -> %zu个机器人",
                         serial_number.c_str(), tcp_map.size());
          }
      } else {
      }

      sqlite3_finalize(stmt);
      return tcp_map;
  }

std::vector<MassageHead> DatabaseManager::getMassageHeadBySerialNumber(const std::string& serial_number) {
    std::vector<MassageHead> heads;

    if (!is_initialized_ || !db_) {
        RCLCPP_ERROR(logger_, "数据库未初始化");
        return heads;
    }

    // 将输入的空格格式转换为数据库中的横杠格式
    std::string db_serial_number = massage_head_manager::utils::hexSpacesToHyphens(serial_number);

    const std::string sql = R"(
        SELECT id, serial_number, name, description,
               image_url, icon_url, config_data, switch_command,
               created_at, updated_at
        FROM massage_head
        WHERE serial_number = ?
        LIMIT 1
    )";

    sqlite3_stmt* stmt = nullptr;
    int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);

    if (result != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "准备查询序列号SQL语句失败: %s", sqlite3_errmsg(db_));
        return heads;
    }

    sqlite3_bind_text(stmt, 1, db_serial_number.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) == SQLITE_ROW) {
        MassageHead head;
        const unsigned char* txt = sqlite3_column_text(stmt, 0);
        head.id = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 1);
        std::string raw_serial = txt ? reinterpret_cast<const char*>(txt) : "";
        head.serial_number = massage_head_manager::utils::hexHyphensToSpaces(raw_serial);

        txt = sqlite3_column_text(stmt, 2);
        head.name = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 3);
        head.description = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 4);
        head.image_url = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 5);
        head.icon_url = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 6);
        head.config_data = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 7);
        head.switch_command = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 8);
        head.created_at = txt ? reinterpret_cast<const char*>(txt) : "";

        txt = sqlite3_column_text(stmt, 9);
        head.updated_at = txt ? reinterpret_cast<const char*>(txt) : "";

        heads.push_back(head);
        RCLCPP_DEBUG(logger_, "通过序列号查询按摩头: serial_number=%s, name=%s",
                    serial_number.c_str(), head.name.c_str());
    } else {
        RCLCPP_WARN(logger_, "未找到序列号对应的按摩头: %s", serial_number.c_str());
    }

    sqlite3_finalize(stmt);
    return heads;
}

// 获取按摩头的switch_command JSON配置
std::string DatabaseManager::getSwitchCommandBySerialNumber(const std::string& serial_number)
{
    // 优先从缓存获取
    auto head = getMassageHeadFromCache(serial_number);
    if (!head.id.empty() && !head.switch_command.empty()) {
        RCLCPP_DEBUG(logger_, "从缓存获取switch_command: serial_number=%s", serial_number.c_str());
        return head.switch_command;
    }
    
    // 缓存未命中，从数据库查询
    if (!is_initialized_ || !db_) {
        RCLCPP_ERROR(logger_, "数据库未初始化");
        return "";
    }
    
    std::string db_serial_number = massage_head_manager::utils::hexSpacesToHyphens(serial_number);
    
    const std::string sql = R"(
        SELECT switch_command
        FROM massage_head
        WHERE serial_number = ?
        LIMIT 1
    )";
    
    sqlite3_stmt* stmt = nullptr;
    int result = sqlite3_prepare_v2(db_, sql.c_str(), -1, &stmt, nullptr);
    
    if (result != SQLITE_OK) {
        RCLCPP_ERROR(logger_, "准备查询switch_command SQL失败: %s", sqlite3_errmsg(db_));
        return "";
    }
    
    sqlite3_bind_text(stmt, 1, db_serial_number.c_str(), -1, SQLITE_TRANSIENT);
    
    std::string switch_command;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        const unsigned char* txt = sqlite3_column_text(stmt, 0);
        switch_command = txt ? reinterpret_cast<const char*>(txt) : "";
        RCLCPP_DEBUG(logger_, "从数据库获取switch_command: serial_number=%s", serial_number.c_str());
    } else {
        RCLCPP_WARN(logger_, "未找到序列号对应的switch_command: %s", serial_number.c_str());
    }
    
    sqlite3_finalize(stmt);
    return switch_command;
}



}