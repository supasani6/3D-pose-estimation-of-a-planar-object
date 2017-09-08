import cv2
import numpy as np
import glob

# Load previously saved data
with np.load('/home/nape/PycharmProjects/calibration2/BBB.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]



def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 20)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 20)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 20)
    img = cv2.drawChessboardCorners(img, (1, 1), corners[0], ret)
    return img


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

for fname in glob.glob('/home/nape/Desktop/programs/chessboard_images/rright2.jpg'):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        R = np.zeros((3, 3), np.float32)
        R[2, 2] = 1.0

        rodRotMat = cv2.Rodrigues(rvecs)
        R[:3, :3] = rodRotMat[0]
        R_inv = np.linalg.inv(R)
        camrot = cv2.Rodrigues(R_inv)
        print "camrot is ", camrot[0]
        mR = np.matrix(R_inv)
        mT = np.matrix(tvecs)
        cam_world_pos = -mR * mT
        print "Cam pose is"
        print cam_world_pos
       

        print "0th corner is :", corners2[0] #changing the index in corners2[] and the corresponding 2D image co-ordinates of the corner in the next line gives 3D position of that chessboard corner

        a = np.array([2088.3347168   , 1446.61755371]) # 2D co-ordinates of 0th corner
        #a = np.array(corners2[0].T)
        b = np.array([1])
        concat = np.concatenate([a,b], 0)
        concat = np.array(concat)[np.newaxis]
        #print "concat vector is"
        #print concat
        #print "transpose", concat.T

	######### rough ##################
    
        dir = mR * np.matrix(concat.T)
        #print "direction vector is ", dir

        unitdir = np.linalg.norm(dir)
        #print "norm is ", unitdir
        dir2 = dir * (1/unitdir)
        #print "unit dir =", dir2

        Xcor = ((-cam_world_pos[2]/dir2[2])*dir2[0])+ cam_world_pos[0]

        #print "Xcor is ", Xcor

        Ycor = ((-cam_world_pos[2]/dir2[2])*dir2[1])+ cam_world_pos[1]

        #print "Ycor is ", Ycor
	#########rough end ###########

        ###### Homography matrix #######
        R2 = np.zeros((3, 2), np.float32)
        R2[:,0]=R[:,0]
        R2[:,1]=R[:,1]

        c = np.array(R2)
        d = np.array(tvecs)
        concat2 = np.concatenate([c, d], 1)
        #print "Cam intrinsics", mtx
        H = mtx * concat2

        H_inv = np.linalg.inv(H)
        #print "H_inv ", H_inv

        #H = H / tvecs[2]
        point3d = np.matrix(H_inv)*np.matrix(concat.T)
        point3d[0] = round(point3d[0]-38)
        point3d[1]= round((point3d[1]-15.5)/2)
        point3d[2]= round(point3d[2] - 0.0433)

        #print "point3d is ", point3d
        ############ Homography end #################

        ############Final Working method ############
        mtx_inv = np.linalg.inv(mtx)


        c = np.array(R)
        d = np.array(tvecs) 
        concat3 = np.concatenate([c, d], 1)
        #print "concat3 is ", concat3

        tempMat = np.matrix(R_inv) * np.matrix(mtx_inv) * np.matrix(concat.T)

        tempMat2 = np.matrix(R_inv) * np.matrix(tvecs)

        s = tempMat2[2,0]
        s = s/tempMat[2,0]

        temp = (s * np.matrix(mtx_inv)*np.matrix(concat.T)) - tvecs


        P = np.matrix(R_inv) * np.matrix(temp)
       
        P[0] = round(P[0])
        P[1] = round(P[1])
        P[2] = round(P[2])

        #print "The 3D position of 0th corner is ", P

        print "Xcor is :" , P[0]
        print "YCor is :", P[1]
        print "ZCor is :", P[2]

        print "success!!"



        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        img = draw(img,corners2,imgpts)
        cv2.namedWindow("Display frame", cv2.WINDOW_NORMAL)
        cv2.imshow("Display frame",img)
        k = cv2.waitKey(0) & 0xff
        if k == 's':
            cv2.imwrite(fname[:6]+'.png', img)


cv2.destroyAllWindows()
