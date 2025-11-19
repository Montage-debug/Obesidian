使用OpenSSL库中的
```cpp
#include <openssl/md5.h>
```

定义以下两个函数，用于计算MD5结果：
```cpp
// 计算字符串的 MD5
static std::string computeMD5(const std::string & input)
{
	unsigned char digest[MD5_DIGEST_LENGTH];
	MD5(reinterpret_cast<const unsigned char *>(input.c_str()), input.length(), digest);
	return bytesToHexString(digest, MD5_DIGEST_LENGTH);
}

// 将字节数组转换为十六进制字符串
static std::string bytesToHexString(const unsigned char * bytes, size_t length)
{
	std::stringstream ss;
	ss << std::hex << std::setfill('0');
	for (size_t i = 0; i < length; ++i)
	{
		ss << std::setw(2) << static_cast<unsigned int>(bytes[i]);
	}
	return ss.str();
}
```

将按要求拼接后的字符串丢给`computeMD5()`就能得到结果

