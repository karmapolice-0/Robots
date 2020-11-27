import numpy as np
from numpy import linalg as LA
from numpy.linalg import LinAlgError


def superpose3d(Xf_orig, Xm_orig, aWeights=None, allow_rescale=False, report_quaternion=False):
    """
    Takes 2 lists of xyz coordinates and attempts
    to superimpose them using rotations, translations
    and (optionally) rescale operations in order to
    minimize the root-mean-squared-distance between them.
    :param Xf_orig: coordinates for the "frozen" object
    :param Xm_orig: coordinates for the "mobile" object
    :param aWeights: optional weigths for the calculation of RMSD
    :param allow_rescale: attempt to rescael mobile point cloud
    :param report_quaternion: report quaternion angle and axis
    :return: (RMSD, optimal_translation, optimal_rotation, optimal_scale_factor)
    """

    Xf_orig = np.array(Xf_orig)
    Xm_orig = np.array(Xm_orig)

    if Xf_orig.shape[0] != Xm_orig.shape[0]:
        raise ValueError ('Inputs should have the same size.')

    N = Xf_orig.shape[0]

    # finding center of mass each object:
    if aWeights is None:
        aWeights = np.full((N, 1), 1.0)
    else:
        aWeights = np.array(aWeights).reshape(N, 1)

    aCenter_f = np.sum(Xf_orig * aWeights, axis=0)
    aCenter_m = np.sum(Xm_orig * aWeights, axis=0)
    sum_weights = np.sum(aWeights, axis=0)

    if sum_weights != 0:
        aCenter_f /= sum_weights
        aCenter_m /= sum_weights
    Xf = Xf_orig - aCenter_f
    Xm = Xm_orig - aCenter_m

    M = np.matmul(Xm.T, (Xf * aWeights))

    # Calculate Q
    Q = M + M.T - 2*np.eye(3)*np.trace(M)

    # Calculate V
    V = np.empty(3)
    V[0] = M[1][2] - M[2][1]
    V[1] = M[2][0] - M[0][2]
    V[2] = M[0][1] - M[1][0]

    # Calculate P
    P = np.zeros((4,4))
    P[:3, :3] = Q
    P[3, :3] = V
    P[:3, 3] = V

    # p is quaternion
    p = np.zeros(4)
    p[3] = 1.0
    pPp = 0.0  # p^T * P * p
    singular = (N < 2)

    try:
        aEigenvals, aaEigenvects = LA.eigh(P)
    except LinAlgError:
        singular = True

    if not singular:
        i_eval_max = np.argmax(aEigenvals)
        pPp = np.max(aEigenvals)
        p[:] = aaEigenvects[:, i_eval_max]

    p /= np.linalg.norm(p)

    aaRotate = np.empty((3, 3))
    aaRotate[0][0] = (p[0]*p[0]) - (p[1]*p[1]) - (p[2]*p[2]) + (p[3]*p[3])
    aaRotate[1][1] = -(p[0]*p[0]) + (p[1]*p[1]) - (p[2]*p[2]) + (p[3]*p[3])
    aaRotate[2][2] = -(p[0]*p[0]) - (p[1]*p[1]) + (p[2]*p[2]) + (p[3]*p[3])
    aaRotate[0][1] = 2 * (p[0]*p[1] - p[2]*p[3])
    aaRotate[1][0] = 2 * (p[0]*p[1] + p[2]*p[3])
    aaRotate[1][2] = 2 * (p[1]*p[2] - p[0]*p[3])
    aaRotate[2][1] = 2 * (p[1]*p[2] + p[0]*p[3])
    aaRotate[0][2] = 2 * (p[0]*p[2] + p[1]*p[3])
    aaRotate[2][0] = 2 * (p[0]*p[2] - p[1]*p[3])

    # from scipy.spatial.transform import Rotation as R
    # the_rotation = R.from_quat(p)
    # aaRotate = the_rotation.as_matrix()

    # Compute E0 from equation 24
    E0 = np.sum((Xf - Xm)**2)
    sum_sqr_dist = max(0, E0 - 2.0 * pPp)

    rmsd = 0.0
    if sum_weights != 0.0:
        rmsd = np.sqrt(sum_sqr_dist / sum_weights)

    # Compute the translational offset
    # RMSD=sqrt((Σ_i  w_i * |X_i - (Σ_j c*R_ij*x_j + T_i))|^2) / (Σ_j w_j))
    #    =sqrt((Σ_i  w_i * |X_i - x_i'|^2) / (Σ_j w_j))
    #  where
    # x_i' = Σ_j c*R_ij*x_j + T_i
    #      = Xcm_i + c*R_ij*(x_j - xcm_j)
    #  and Xcm and xcm = center_of_mass for the frozen and mobile point clouds
    #                  = aCenter_f[]       and       aCenter_m[],  respectively
    # Hence:
    #  T_i = Xcm_i - Σ_j c*R_ij*xcm_j  =  aTranslate[i]
    aTranslate = aCenter_f - np.matmul(aaRotate, aCenter_m).T.reshape(3,)

    if report_quaternion:
        q = np.empty(4)
        q[0] = p[3]
        q[1] = p[0]
        q[2] = p[1]
        q[3] = p[2]
        return rmsd, q, aTranslate, 1.0
    else:
        return rmsd, aaRotate, aTranslate, 1.0
