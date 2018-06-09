import cv2
import numpy as np
import math

def transfer_Grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def transfer_Gaussianblur(Gray_img, kernel_size):
    return cv2.GaussianBlur(Gray_img, (kernel_size, kernel_size), 0)

def detect_Canny_Edge(Blur_img, minTreshold, maxTreshold):
    return cv2.Canny(Blur_img, minTreshold, maxTreshold)


def extraction_ROI(Canny_img, want_roi):
    mask = np.zeros_like(Canny_img)
    cv2.fillPoly(mask, want_roi, [255,255,255])#3채널 영상으로 컬러값 넘겨줌
    ROI_image = cv2.bitwise_and(Canny_img, mask)
    return ROI_image

def extraction_HoughlinesP(Roi_img, rho, angle, threshold, minLineLength, maxLineGap): # 허프 변환
    lines =cv2.HoughLinesP(Roi_img, rho, angle, threshold,minLineLength, maxLineGap)
    if lines is not None:
        for item in lines:
                lines = item
                return lines

def fit_line(img,lines):

        try :
            lines = lines.reshape(lines.shape[0]*2,2)
            rows,cols = img.shape[:2]
            output = cv2.fitLine(lines,cv2.DIST_L2,0, 0.01, 0.01)
            vx, vy, x, y = output[0], output[1], output[2], output[3]
            print(output)
            try:
                x1, y1, x2, y2= int(((rows-int(rows*0.1))-y)/vy*vx + x) , rows-int(rows*0.1), int(((rows/2+1)-y)/vy*vx + x) , int(rows/2+1)
                # cv2.line(img,(x1,y1),(x2,y2),[255,0,0],3)
                result = [x1,y1,x2,y2]
                # print(result)

            except:
                x1, y1, x2, y2= int(((rows-int(rows*0.1))-y)/(vy+0.001)*(vx+0.001) + x) , rows-int(rows*0.1), int(((rows/2+1)-y)/(vy+0.001)*(vx+0.001) + x) , int(rows/2+1)
                result = [x1,y1,x2,y2]
        except:
            result = lines
        return result


def draw_line(img,line,color=[255,0,0],tick=5):
    try:
        cv2.line(img,(line[0],line[1]),(line[2],line[3]),color,tick)
    except:
        pass
def Shi_Tomasi_corner(img,cornerNum = 2, threshold = 0.01,quility =15):
    try:

        return cv2.goodFeaturesToTrack(img,cornerNum,threshold,quility)
    except:
        pass
Capture = cv2.VideoCapture('2017_03_19_1(1).mp4')
f = open('line.txt','w')
min_treshold_val = 100
max_treshold_val = 160#canny 정도
stop_line_num = 0
line_i = 0
while(1):
    is_img_right, img = Capture.read() #주행하다 영상이 꺼졌을때 예외 처리 None값 안들어가게~~
    if is_img_right:
        height, width = img.shape[:2]
        Gray_img = transfer_Grayscale(img)
        Blur_img = transfer_Gaussianblur(Gray_img, 3)
        Canny_img = detect_Canny_Edge(Blur_img,min_treshold_val, max_treshold_val)
        want_roi = np.array([[(width*0.03,height*0.3),(width*0.08, height*0.8), (width-(width*0.08), height*0.8), (width-(width*0.03),height*0.3)]], dtype=np.int32)
        Roi_img = extraction_ROI(Canny_img,want_roi)
        lines = extraction_HoughlinesP(Roi_img,1,np.pi/180, 10,10,15)#(허프라인 검출할 이미지, 반지름r(0~1), 1도(np.pi=180),허프라인정도, 최소임계,최대임계)
        zero_img = np.zeros((img.shape[0], img.shape[1],3),dtype = np.uint8)
        zero_img_two = np.zeros((img.shape[0], img.shape[1],3),dtype = np.uint8)
        row, cols = img.shape[:2]
        if lines is not None:
            try:
                print( left_line.append(temp_left_line),'abc-----------')
            except:
                pass
            first_slope = np.arctan2(lines[:,1]-lines[:,3],lines[:,0]-lines[:,2])*180/np.pi
            if np.all(abs(first_slope) >=180 and abs(first_slope)<185) :
                    corner = np.squeeze(Shi_Tomasi_corner(Gray_img))
                    if np.any(abs(int(corner[0][0]) -  int(lines[0][0]))<4  or abs(int(corner[1][0]) - int(lines[0][2]))<4):
                        try:
                            if abs( abs(corner[1][0] - corner[0][0]) - avg_line_width )<20:
                                print(corner,lines,'corner, lines')
                                print(corner[0][0],lines[0][0],lines[0][0]-lines[0][0],'corner//')
                                stop_line = lines
                                stop_line_slope = first_slope #해리스 추가하기
                                fit = fit_line(zero_img,stop_line)
                                draw_line(zero_img,fit)
                                data1 = ('[%d]--------------------------------------!!!stop!!!--------------------------------- : %s\n'%(stop_line_num,stop_line))
                                f.write(data1)
                                stop_line_num = stop_line_num +1
                                print(stop_line,'stop_line')
                                print(stop_line_slope,'stop_line_slope')
                        except:
                            pass


            elif np.all(abs(first_slope)>140 and abs(first_slope)<175):
                    data2 = ('[%d]left_line : %s\n'%(line_i,lines))
                    f.write(data2)
                    line_i = line_i+1
                    try:
                        avg_line_width = abs(left_line[0][0] - right_line[0][0])
                    except:
                        pass
                    if first_slope>0:
                        try:
                                if np.all(avg_line_width - abs(lines[0][2] - lines[0][2])>5):
                                    left_line = temp_left_line
                                    left_line_slope = temp_left_slope
                                    fit = fit_line(zero_img,left_line)
                                    draw_line(zero_img,fit)
                                    print(left_line,'left_line__tmp')
                                else:
                                    left_line = lines
                                    temp_left_line = left_line
                                    left_line_slope = first_slope
                                    temp_left_slope = left_line_slope
                                    fit = fit_line(zero_img,left_line)
                                    draw_line(zero_img,fit)
                                    print(left_line,'left_line')
                                    print(left_line_slope,'left_line_slope')
                        except:
                                left_line = lines
                                temp_left_line = left_line
                                left_line_slope = first_slope
                                temp_left_slope = left_line_slope
                                fit = fit_line(zero_img,left_line)
                                draw_line(zero_img,fit)
                                print(left_line,'left_line')
                                print(left_line_slope,'left_line_slope')
                        try:
                                avg_line_width = abs(left_line[0][0] - right_line[0][0])
                        except:
                                pass

                    else:
                        data3 = ('[%d]right_line : %s\n'%(line_i,lines))
                        f.write(data3)
                        line_i = line_i+1

                        try:
                            if np.all(avg_line_width - abs(lines[0][2] - lines[0][2])>5 ):
                                right_line = temp_right_line
                                right_line_slope = temp_right_slope
                                fit = fit_line(zero_img,right_line)
                                draw_line(zero_img,fit)
                                print(right_line,'right_line__tmp')
                                print(right_line_slope,'right_line_slope__tmp')

                            else:
                                right_line = lines
                                temp_right_line = right_line
                                right_line_slope = first_slope
                                temp_right_slope = right_line_slope
                                fit = fit_line(zero_img,right_line)
                                draw_line(zero_img,fit)
                                print(right_line,'right_line')
                                print(right_line_slope,'right_line_slope')
                        except:
                            right_line = lines
                            temp_right_line = right_line
                            right_line_slope = first_slope
                            temp_right_slope = right_line_slope
                            fit = fit_line(zero_img,right_line)
                            draw_line(zero_img,fit)
                            print(right_line,'right_line')
                            print(right_line_slope,'right_line_slope')

            else:
                try:
                    fit1 = fit_line(zero_img,temp_left_line)
                    draw_line(zero_img,fit1)
                    fit2 = fit_line(zero_img,temp_right_line)
                    draw_line(zero_img,fit2)
                    print(first_line, first_slope,"? line")
                    print(left_line,right_line,"left,right except")
                except:
                    print(lines, first_slope,"? line")
                    try:
                        print(left_line,right_line,"left,right except")
                    except:
                        pass
                try:
                    data6 = ('[%d]left,right__Avg : %s, %s\n'%(line_i,temp_left_line,temp_right_line))
                    f.write(data6)
                    line_i = line_i+1
                except:
                    pass
                try:
                    print(temp_left_line, temp_right_line,'temp left righ  ?line---')
                except:
                    pass

    line_i = line_i+1
    result = cv2.addWeighted(zero_img,1,img,1,0)

    cv2.imshow('aa',result)

    if cv2.waitKey(1)&0xFF == ord('q'):
            break;

f.close()
Capture.release()
cv2.destroyAllWindows()
