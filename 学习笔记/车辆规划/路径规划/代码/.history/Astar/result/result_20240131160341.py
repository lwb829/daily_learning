from PIL import Image
import os


def create_gif(image_folder, output_gif_path, duration=500):
    images = []

    # 遍历文件夹中的所有PNG图像
    for filename in sorted(os.listdir(image_folder)):
        if filename.endswith(".png"):
            img_path = os.path.join(image_folder, filename)
            images.append(Image.open(img_path))

    # 设置GIF动图的帧持续时间（以毫秒为单位）
    durations = [duration] * len(images)

    # 保存为GIF
    images[0].save(output_gif_path, save_all=True,
                   append_images=images[1:], duration=durations, loop=0)


# 调用函数，传入图像文件夹路径和输出的GIF路径
create_gif("Astar\result", "result.gif")
