import csv
import os
import xlsxwriter
import math

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return yaw, pitch, roll

def main():
    imu_files = ["Quat_imu_left.xlsx", "Quat_imu_right.xlsx", "Quat_imu_frame.xlsx"]
    filename = ["imu_left.xlsx", "imu_right.xlsx", "imu_frame.xlsx"]
    for i in range(0,3):
        workbook = xlsxwriter.Workbook(filename[i])
        worksheet = workbook.add_worksheet()
        row = 0
        col = 0
        f = open(imu_files[i])
        reader = csv.reader(f)
        next(reader, None)
        worksheet.write(row, col, "Time")
        worksheet.write(row, col + 1, "Yaw")
        worksheet.write(row, col + 2, "Pitch")
        worksheet.write(row, col + 3, "Roll")
        worksheet.write(row, col + 4, "Angular Velocity x")
        worksheet.write(row, col + 5, "Angular Velocity y")
        worksheet.write(row, col + 6, "Angular Velocity z")
        worksheet.write(row, col + 7, "Linear Acceleration x")
        worksheet.write(row, col + 8, "Linear Acceleration y")
        worksheet.write(row, col + 9, "Linear Acceleration z")
        row += 1
        for time, seq, stamp, frame, x, y, z, w, c0, c1, c2, c3, c4, c5, c6, c7, c8, \
            ang_x, ang_y, ang_z, ang_c0, ang_c1, ang_c2, ang_c3, ang_c4, ang_c5, \
            ang_c6, ang_c7, ang_c8, lin_x, lin_y, lin_z, lin_c0, lin_c1, lin_c2, \
            lin_c3, lin_c4, lin_c5, lin_c6, lin_c7, lin_c8 in reader:

            yaw, pitch, roll = quaternion_to_euler(float(x), float(y), float(z), float(w))

            worksheet.write(row, col, time)
            worksheet.write(row, col + 1, yaw)
            worksheet.write(row, col + 2, pitch)
            worksheet.write(row, col + 3, roll)
            worksheet.write(row, col + 4, ang_x)
            worksheet.write(row, col + 5, ang_y)
            worksheet.write(row, col + 6, ang_z)
            worksheet.write(row, col + 7, lin_x)
            worksheet.write(row, col + 8, lin_y)
            worksheet.write(row, col + 9, lin_z)
            row +=1
        workbook.close()

if __name__ == "__main__":
    main()