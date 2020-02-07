import numpy as np

# Estimate a rigid transform between 2 set of points of equal length
# through singular value decomposition(svd)
# return a rotation and a transformation matrix
def rigid_transform_3D(A, B):

    assert len(A) == len(B)
    A=  np.asmatrix(A)
    B=  np.asmatrix(B)
    N = A.shape[0]; 

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))
    H = AA.T * BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T * U.T

    # reflection case
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T
    # rotation matrix
    R = np.array(R)
    # translation
    t = (np.array(t).T)[0]

    # B = np.dot(R, A.T).T+t
    # 4 by 4 homogeneous transformation matrix from rotation and translation
    transform = [[R[0][0],R[0][1],R[0][2],t[0]],
                 [R[1][0],R[1][1],R[1][2],t[1]],
                 [R[2][0],R[2][1],R[2][2],t[2]],
                 [0,0,0,1]]

    return (R,t)



B = np.array([[0.3, 0.5, 0.217],
              [0.3, 0.6, 0.217],
              [0.4, 0.6, 0.217],
              [0.4, 0.5, 0.217],
              [0.4, 0.5, 0.317],
              [0.4, 0.4, 0.317],
              [0.4, 0.4, 0.217],
              [0.3, 0.4, 0.217],
              [0.2, 0.4, 0.317],
              [0.5, 0.4, 0.217]])

A = np.array([[-0.047241900116205215, 0.0775892585515976, 0.8319157361984253],
              [-0.11193868517875671, 0.09636501222848892, 0.7486035227775574],
              [-0.031776949763298035, 0.10616994649171829, 0.6926715970039368],
              [0.034347910434007645, 0.08743490278720856, 0.7781679630279541],
              [0.03698727488517761, 0.18554580211639404, 0.7891932725906372],
              [0.1001746654510498, 0.16669361293315887, 0.8706247806549072],
              [0.09763826429843903, 0.06988470256328583, 0.8485807180404663],
              [0.01804971881210804, 0.05828729644417763, 0.921383261680603],
              [-0.05895867943763733, 0.14373746514320374, 0.9906587600708008],
              [0.1784733235836029, 0.07979770749807358, 0.7865796685218811]])

R, t = rigid_transform_3D(A, B)

print('Rotation: ', R)
print('Translation: ', t)

diff = 0

for i in range(len(A)):
    new_A = A[i]
    new_B = np.dot(R, new_A.T).T+t
    print("#########", i+1, "##############")
    print("observed: ", B[i])
    print("predicted: ", new_B)
    dist = np.sqrt(np.sum((B[i]-new_B)**2)) * 100
    diff += dist

print("Average difference: ", diff/len(A))








