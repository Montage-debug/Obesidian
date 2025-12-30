
https://github.com/yhirose/cpp-httplib/
该项目是单头文件库，下载后得到
![[httplib.h]]

以调用百度翻译API为例
==创建client==
```cpp
httplib::Client cli("fanyi-api.baidu.com");
```

==通常需要配置以下两种参数形式==

- ==params==**（参数）主要用于定义“你要什么”**
创建params参数：
```cpp
httplib::Params params = {
	{"q", "Apple"},
	{"from", "en"},
	{"to", "zh"},
	{"appid", "20230411001637303"},
	{"salt", "12345"},
	{"appkey", "WSWouPFcBO7ZyuJnF9YS"},
	{"sign", md5_str},
};
```
通常作为显性的参数显示在URL中
比如以上信息会显示为：
`https://fanyi-api.baidu.com/api/trans/vip/translate?q=apple&from=en&to=zh&appid=2015063000000001&salt=65478&sign=a1a7461d92e5194c5cae3182b5b24de1`

- ==headrs==**（头信息）主要用于描述“这次请求/响应的具体情况是怎样的”**
以`寻艾AI中医平台`为例
```cpp
httplib::Headers headers = {
	{"secretId", secret_id},
	{"timestamp", times_tampe},
	{"sign", md5_str},
};
```
不直接显示在URL中


==MD5加密sign签名==
以上两个例子都用到了`md5_str`这个变量
涉及到MD5（Message Digest Algorithm 5）
是一种常用的**哈希函数**
通常不同的API会有不同的MD5函数输入的要求
以百度为例：
```cpp
stringstream res;
res << "appid" << "Apple" << "随机数" << "appkey";
```
拼接字符串后，丢给MD5计算函数
```cpp
string md5_str
md5_str = computeMD5(res.str());
```
计算函数详见：[[MD5的使用]]