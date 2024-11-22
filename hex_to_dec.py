def hex_to_decimal(hex_string):
    # 去掉输入字符串中的空格
    hex_string = hex_string.replace(" ", "")
    
    # 将十六进制字符串转换为十进制整数
    decimal_value = int(hex_string, 16)
    
    return decimal_value

if __name__ == "__main__":
    while True:
        # 从控制台获取输入
        hex_input = input("请输入十六进制字符串（例如：0F 04 8C），或输入 'exit' 退出：")
        
        # 检查是否退出
        if hex_input.lower() == 'exit':
            break
        
        try:
            # 转换为十进制
            decimal_output = hex_to_decimal(hex_input)
            
            # 打印结果
            print(f"十六进制输入: {hex_input}")
            print(f"十进制输出: {decimal_output}")
        except ValueError:
            print("输入的十六进制字符串无效，请重新输入。")