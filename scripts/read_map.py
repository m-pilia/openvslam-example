#!/usr/bin/env python3

import os
import sys

from threading import Thread

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
import msgpack
import numpy as np
import open3d as o3d
import pptk
import scipy.spatial.transform as sst

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import QObject, QUrl, pyqtSignal, pyqtProperty
from PyQt5.QtQuick import QQuickView
from PyQt5.QtWidgets import QWidget

QtCore.Signal = QtCore.pyqtSignal


def _change_frame(a):
    """ (x, y, z) <- (x, z, -y) """
    return np.concatenate((a[:, [0]], a[:, [2]], np.negative(a[:, [1]])), axis=1)


class MapInfo(QObject):
    loadingChanged = pyqtSignal()
    filenameChanged = pyqtSignal()
    keypointCountChanged = pyqtSignal()
    keyframeCountChanged = pyqtSignal()

    visualiseTrajectory = pyqtSignal()
    visualiseCloud = pyqtSignal()
    exportCloud = pyqtSignal('QString')

    def __init__(self, parent=None):
        QObject.__init__(self, parent)
        self._loading = False
        self._filename = None
        self._keypoint_count = 0
        self._keyframe_count = 0

    @pyqtProperty('bool', notify=loadingChanged)
    def loading(self):
        return self._loading

    @loading.setter
    def loading(self, status):
        self._loading = status
        self.loadingChanged.emit()

    @pyqtProperty('int', notify=keyframeCountChanged)
    def keyframeCount(self):
        return self._keyframe_count

    @keyframeCount.setter
    def keyframeCount(self, n):
        self._keyframe_count = n
        self.keyframeCountChanged.emit()

    @pyqtProperty('int', notify=keypointCountChanged)
    def keypointCount(self):
        return self._keypoint_count

    @keypointCount.setter
    def keypointCount(self, n):
        self._keypoint_count = n
        self.keypointCountChanged.emit()

    @pyqtProperty('QString', notify=filenameChanged)
    def filename(self):
        return self._filename

    @filename.setter
    def filename(self, filename):
        self._filename = filename.replace('file://', '')
        self.filenameChanged.emit()


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.setWindowTitle('Read OpenVSLAM map')

        self.mapInfo = MapInfo()

        qml_view = QQuickView()
        qml_view.rootContext().setContextProperty('mapInfo', self.mapInfo)
        qml_view.setSource(QUrl(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'read_map.qml')))
        qml_view.setResizeMode(QQuickView.SizeRootObjectToView)
        self.qml_gui = qml_view.rootObject()

        qml_view_container = QWidget.createWindowContainer(qml_view, self)
        qml_view_container.setMinimumSize(800, 200)
        self.setCentralWidget(qml_view_container)

        self.mapInfo.filenameChanged.connect(self.read_data)
        self.mapInfo.visualiseCloud.connect(self.pptk)
        self.mapInfo.visualiseTrajectory.connect(self.pyplot)
        self.mapInfo.exportCloud.connect(self.export_cloud)

        self.data = None
        self.cloud = None
        self.trajectory = None

    def _get_landmarks(self):
        landmarks = self.data[b'landmarks'].values()
        cloud = np.ndarray([len(landmarks), 3], dtype=np.double)

        for i, l in enumerate(landmarks):
            cloud[i, :] = np.array(l[b'pos_w'])

        self.cloud = _change_frame(cloud)
        self.cloud_colour = np.repeat([[0.7, 0.7, 0.7]], cloud.shape[0], axis=0)
        self.mapInfo.keypointCount = cloud.shape[0]

    def _get_trajectory(self):
        keyframes = self.data[b'keyframes']
        keyframes = {ki: keyframes[b'%d' % ki] for ki in sorted([int(k) for k in keyframes.keys()])}
        trajectory = np.ndarray([len(keyframes), 3], dtype=np.double)

        for i, (k, f) in enumerate(keyframes.items()):
            transform = -sst.Rotation.from_quat(f[b'rot_cw']).as_dcm().transpose()
            trajectory[i, :] = np.matmul(transform, f[b'trans_cw'])

        n = trajectory.shape[0]
        self.mapInfo.keyframeCount = n

        self.trajectory = _change_frame(trajectory)
        self.trajectory_colour = np.array([[1 - x / n, 0.0, x / n] for x in range(1, n + 1)])

    def read_data(self):
        def thread_function():
            with open(self.mapInfo.filename, 'rb') as f:
                self.data = msgpack.unpack(f)

            self._get_landmarks()
            self._get_trajectory()
            self.mapInfo.loading = False

        thread = Thread(target=thread_function)
        self.mapInfo.loading = True
        thread.start()

    def export_cloud(self, filename):
        Thread(target=lambda: np.savetxt(filename.replace('file://', ''), self.cloud, delimiter=',')).start()

    def open3d(self):
        if not self.data:
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.concatenate((self.cloud, self.trajectory), axis=0))
        pcd.colors = o3d.utility.Vector3dVector(np.concatenate((self.cloud_colour, self.trajectory_colour), axis=0))
        o3d.visualization.draw_geometries([pcd])

    def pyplot(self):
        if not self.data:
            return

        fig = plt.figure()
        ax = plt3d.Axes3D(fig)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.scatter(self.trajectory[:, 0], self.trajectory[:, 1], self.trajectory[:, 2])
        vrange = [np.min(self.trajectory), np.max(self.trajectory)]
        ax.auto_scale_xyz(vrange, vrange, vrange)
        plt.show()

    def pptk(self):
        if not self.data:
            return

        v = pptk.viewer(np.concatenate((self.cloud, self.trajectory), axis=0))
        v.attributes(np.concatenate((self.cloud_colour, self.trajectory_colour), axis=0))
        v.set(lookat=np.mean(self.cloud, axis=0), point_size=0.005)


if __name__ == '__main__':
    application = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(application.exec_())
