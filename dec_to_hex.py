def decimal_to_hex(decimal_number):
    # 将十进制整数转换为十六进制字符串
    hex_string = hex(decimal_number).upper().replace("0X", "")
    
    return hex_string

if __name__ == "__main__":
    while True:
        # 从控制台获取输入
        decimal_input = input("请输入十进制数，或输入 'exit' 退出：")
        
        # 检查是否退出
        if decimal_input.lower() == 'exit':
            break
        
        try:
            # 转换为十六进制
            decimal_number = int(decimal_input)
            hex_output = decimal_to_hex(decimal_number)
            
            # 打印结果
            print(f"十进制输入: {decimal_number}")
            print(f"十六进制输出: {hex_output}")
        except ValueError:
            print("输入的十进制数无效，请重新输入。")