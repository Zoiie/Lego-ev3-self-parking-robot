import csv
import os

import matplotlib.pyplot as plt
import numpy as np

bound_x_combine=[]
bound_y_combine=[]

def draw_vehicle0(exampleFile,number):
    exampleReader_vehicle = csv.reader(exampleFile)
    exampleData_vehicle = list(exampleReader_vehicle)
    length_zu_vehicle = len(exampleData_vehicle)

    x1 = list()
    y1 = list()
    for k in range(0, length_zu_vehicle):
        x1.append(float(exampleData_vehicle[k][0]))
        bound_x_combine.append(float(exampleData_vehicle[k][0]))
        y1.append(float(exampleData_vehicle[k][1]))
        bound_y_combine.append(float(exampleData_vehicle[k][1]))

    print(x1)
    # plt.plot(x1,y1, linewidth=1,label="vehicle",color="red")
    plt.plot(x1,y1,"o",label="boundary")

def main():
    exampleFile_vehicle1 = open('./test/0leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle2 = open('./test/1leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle3 = open('./test/2leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle4 = open('./test/3leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle5 = open('./test/4leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle6 = open('./test/5leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle7 = open('./test/6leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle8 = open('./test/7leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle9 = open('./test/8leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle10 = open('./test/9leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle11 = open('./test/10leftboundtest.csv')  # 打开csv文件
    exampleFile_vehicle12 = open('./test/11leftboundtest.csv')  # 打开csv文件

    exampleFile_vehicle13 = open('./test/0rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle14 = open('./test/1rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle15 = open('./test/2rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle16 = open('./test/3rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle17 = open('./test/4rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle18 = open('./test/5rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle19 = open('./test/6rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle20 = open('./test/7rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle21 = open('./test/8rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle22 = open('./test/9rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle23 = open('./test/10rightboundtest.csv')  # 打开csv文件
    exampleFile_vehicle24 = open('./test/11rightboundtest.csv')  # 打开csv文件

    draw_vehicle0(exampleFile_vehicle1,1)
    # draw_vehicle0(exampleFile_vehicle2,1)
    # draw_vehicle0(exampleFile_vehicle3,1)
    draw_vehicle0(exampleFile_vehicle4,1)
    # draw_vehicle0(exampleFile_vehicle5,1)
    draw_vehicle0(exampleFile_vehicle6,1)
    # draw_vehicle0(exampleFile_vehicle7,1)
    # draw_vehicle0(exampleFile_vehicle8,1)
    # draw_vehicle0(exampleFile_vehicle9,1)
    # draw_vehicle0(exampleFile_vehicle10,1)
    # draw_vehicle0(exampleFile_vehicle11,1)
    # draw_vehicle0(exampleFile_vehicle12,1)
    draw_vehicle0(exampleFile_vehicle13,1)
    draw_vehicle0(exampleFile_vehicle14,1)
    draw_vehicle0(exampleFile_vehicle15,1)
    draw_vehicle0(exampleFile_vehicle16,1)
    draw_vehicle0(exampleFile_vehicle17,1)
    draw_vehicle0(exampleFile_vehicle18,1)
    # draw_vehicle0(exampleFile_vehicle19,1)
    draw_vehicle0(exampleFile_vehicle20,1)
    draw_vehicle0(exampleFile_vehicle21,1)
    # draw_vehicle0(exampleFile_vehicle22,1)
    # draw_vehicle0(exampleFile_vehicle23,1)
    draw_vehicle0(exampleFile_vehicle24,1)

    # for i in range(0,12):
    #     # f = csv.reader(open(str(i)+'out_path.csv','r'))
    #     exampleFile_vehicle = open(str(i)+'leftboundtest.csv')  # 打开csv文件
    #     draw_vehicle0(exampleFile_vehicle, i)
    #     exampleFile_vehicle1 = open(str(i)+'rightboundtest.csv')  # 打开csv文件
    #     draw_vehicle0(exampleFile_vehicle1, i)

    print("kaishi:\n",bound_x_combine,"\n",bound_y_combine)
    print(np.shape(bound_x_combine),np.shape(bound_y_combine))

    # 把边界写入文件
    bound_combine=np.c_[bound_x_combine,bound_y_combine]
    f = open('E:/pycharm_codes/boundary_combine/boundary_combine.csv', 'w', newline='')
    writer = csv.writer(f)
    for j in bound_combine:
        writer.writerow(j)

    # plt.show()

if __name__=="__main__":
    main()
    plt.show()