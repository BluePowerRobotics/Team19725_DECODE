            // 读取图像
            Mat frame = Imgcodecs.imread(imagePath);
            if (frame.empty()) {
                System.err.println("Failed to load image: " + imagePath);
                continue;
            }
            
            // 复制原始图像用于显示
            Mat resultFrame = frame.clone();
            
            // 使用两个匹配器处理图像
