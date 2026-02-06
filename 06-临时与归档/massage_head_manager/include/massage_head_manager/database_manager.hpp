#ifndef MASSAGE_HEAD_MANAGER_DATABASE_MANAGER_HPP
#define MASSAGE_HEAD_MANAGER_DATABASE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include <sqlcipher/sqlite3.h>
#include <nlohmann/json.hpp>
#include <map>
#include <mutex>

namespace massage_head_manager
{
    struct MassageHead
    {
        std::string id;               // UUID格式
        std::string serial_number;
        std::string name;
        std::string description;
        std::string image_url;
        std::string icon_url;
        std::string config_data;
        std::string switch_command;   // 开关控制命令JSON
        std::string created_at;
        std::string updated_at;
    };  

   // 按摩头参数结构体
struct MassageHeadParameters
{
    std::string name;
    float tcp_offset;                    // TCP偏移量
    int intensity;                       // 力度 (1-10级)
    int rolling_speed;                   // 滚动速度 (1-10级)
    std::string stimulation_mode;        // 刺激模式 (continuous/intermittent/wave)
    float temperature;                   // 温度 (28-42℃)
    bool enable;                         // 启用状态
};

struct TcpPosition {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
};


class DatabaseManager 
{
    public:
            explicit DatabaseManager(const rclcpp::Logger& logger);
            ~DatabaseManager();
        
            // 初始化数据库
    bool initialize(const std::string& db_path = "");
    std::map<std::string, TcpPosition> getTcpMapByMassageHeadName(const std::string& name);

            // 按摩头管理
    std::vector<MassageHead> getAllMassageHead();
    bool saveMassageHead(const MassageHead& config);

    // 通过功能码查询按摩头信息 - 已移除function_code字段
    // MassageHead getMassageHeadByFunctionCode(const std::string& function_code);
    // std::vector<MassageHead> getMassageHeadByFunctionCodes(const std::vector<std::string>& function_codes);
    std::vector<MassageHead> getMassageHeadBySerialNumber(const std::string& serial_number);

    //根据serial_number获取TCP数据
    std::map<std::string, TcpPosition> getMassageHeadTcpBySerialNumber(const std::string& serial_number);
    // 根据function_code获取TCP数据 - 已移除function_code字段
    // std::map<std::string, TcpPosition> getMassageHeadTcpByFunctionCode(const std::string& function_code);
    // config_data JSON 字符串解析 TCP 字段
    static std::map<std::string, TcpPosition> parseTcpFromConfigData(const std::string& config_data_json);
    // 从 config_data JSON 字符串解析 payload 字段
    static double parsePayloadFromConfigData(const std::string& config_data_json);
    
    // 获取按摩头的switch_command JSON配置
    std::string getSwitchCommandBySerialNumber(const std::string& serial_number);
        
    // 数据库维护
    void close();

    // 缓存查找方法（快速访问）
    MassageHead getMassageHeadFromCache(const std::string& serial_number);
    // MassageHead getMassageHeadFromCacheByFunctionCode(const std::string& function_code); - 已移除function_code字段
    std::map<std::string, TcpPosition> getTcpFromCache(const std::string& serial_number);



        
private:
    rclcpp::Logger logger_;
    sqlite3* db_;
    std::string db_path_;
    bool is_initialized_;

    std::vector<MassageHead> cached_heads_;
    std::map<std::string, std::map<std::string, TcpPosition>> cached_head_tcps_; // serial_number -> (robot -> xyz)
    std::map<std::string, MassageHead> cached_by_serial_;     // serial_number -> MassageHead
    // std::map<std::string, MassageHead> cached_by_function_;   // function_code -> MassageHead - 已移除function_code字段

    std::mutex db_mutex_; // 若节点多线程访问
        
    // // 数据库创建和初始化
    // bool createTables();
    // bool createMassageHeadTable();
    // bool createOperationLogTable();
    // bool createRunningStateTable();
       
    // // SQL执行辅助函数
    // bool executeSQL(const std::string& sql);
    // sqlite3_stmt* prepareStatement(const std::string& sql);
    // bool executeStatement(sqlite3_stmt* stmt);

};
   

};

#endif

