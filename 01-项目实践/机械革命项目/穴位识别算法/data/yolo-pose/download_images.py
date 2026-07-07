import os
import requests
from urllib.parse import urlparse
from pathlib import Path
from bs4 import BeautifulSoup


def download_image(url, save_path):
    """
    下载单张图片并保存为jpg格式
    
    Args:
        url (str): 图片URL
        save_path (str): 保存路径
    """
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        
        # 确保保存目录存在
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        # 保存为jpg格式
        with open(save_path, 'wb') as f:
            f.write(response.content)
        print(f"成功下载: {save_path}")
        return True
    except Exception as e:
        print(f"下载失败 {url}: {str(e)}")
        return False


def extract_image_urls(page_url, num_images):
    """
    从网页中提取前 num_images 个图片URL
    
    Args:
        page_url (str): 网页URL
        num_images (int): 需要提取的图片数量
        
    Returns:
        list: 图片URL列表
    """
    try:
        headers = {'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'}
        response = requests.get(page_url, headers=headers, timeout=10)
        response.raise_for_status()
        
        soup = BeautifulSoup(response.text, 'html.parser')
        img_tags = soup.find_all('img')
        
        urls = []
        for img in img_tags:
            img_url = img.get('src') or img.get('data-src')
            if img_url:
                # 处理相对路径
                if img_url.startswith('//'):
                    img_url = 'https:' + img_url
                elif img_url.startswith('/'):
                    parsed_url = urlparse(page_url)
                    img_url = f"{parsed_url.scheme}://{parsed_url.netloc}{img_url}"
                urls.append(img_url)
                
                if len(urls) >= num_images:
                    break
                    
        return urls
    except Exception as e:
        print(f"提取图片链接失败: {str(e)}")
        return []


def download_images(page_url, num_images, save_dir):
    """
    从指定网页下载指定数量的图片到指定目录
    
    Args:
        page_url (str): 包含图片的网页URL
        num_images (int): 要下载的图片数量
        save_dir (str): 保存目录
    """
    # 提取图片链接
    image_urls = extract_image_urls(page_url, num_images)
    if not image_urls:
        print("未找到可下载的图片链接")
        return
        
    # 确保保存目录存在
    Path(save_dir).mkdir(parents=True, exist_ok=True)
    
    successful_downloads = 0
    for i, url in enumerate(image_urls):
        if successful_downloads >= num_images:
            break
            
        # 生成保存文件名
        filename = f"image_{successful_downloads + 1:04d}.jpg"
        save_path = os.path.join(save_dir, filename)
        
        if download_image(url, save_path):
            successful_downloads += 1
        else:
            print(f"跳过第 {i + 1} 张图片下载")
    
    print(f"总共成功下载 {successful_downloads} 张图片")


def main():
    # 直接在 main 中定义参数，不再使用命令行传参
    page_url = "https://www.bing.com/images/search?q=Asian+women+showing+their+belly+button&qs=n&form=QBIR&sp=-1&lq=0&sc=9-0&cvid=A3AECAB68D48413B862589EC09B6BADB&first=1&cw=2473&ch=1277"  # 替换为实际图片页面URL
    num_images = 20  # 默认下载1张
    save_dir = "/home/cjk/myproject/Dataset/download_images"  # 默认保存目录

    download_images(page_url, num_images, save_dir)


if __name__ == "__main__":
    main()