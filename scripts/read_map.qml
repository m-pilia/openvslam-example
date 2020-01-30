import QtQuick 2.2
import QtQuick.Controls 2.2
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3

Item {
    id: root

    DropArea {
        anchors.fill: parent
        onDropped: {
            mapInfo.filename = drop.urls[0]
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 5

        RowLayout {
            Layout.fillWidth: true

            Button {
                FileDialog {
                    id: loadMapFileDialog
                    title: "Please choose a file"
                    folder: shortcuts.home
                    nameFilters: [ "OpenVSLAM maps (*.msg)", "All files (*)" ]

                    onAccepted: {
                        mapInfo.filename = loadMapFileDialog.fileUrls[0]
                    }
                }

                text: 'Load map'
                onClicked: loadMapFileDialog.open()
            }

            BusyIndicator {
                running: mapInfo.loading
            }

            Button {
                text: 'Visualise key points'
                onClicked: mapInfo.visualiseCloud()
            }

            Button {
                text: 'Visualise key frames'
                onClicked: mapInfo.visualiseTrajectory()
            }

            Button {
                FileDialog {
                    id: exportCloudFileDialog
                    title: "Please choose a file"
                    folder: shortcuts.home
                    nameFilters: [ "Comma-separated values file (*.csv)", "All files (*)" ]
                    selectExisting: false

                    onAccepted: {
                        mapInfo.exportCloud(loadMapFileDialog.fileUrls[0])
                    }
                }

                text: 'Export key points'
                onClicked: exportCloudFileDialog.open()
            }
        }

        GridLayout {
            columns: 2

            Label { text: "File: "; Layout.alignment: Qt.AlignRight }
            Label { text: mapInfo.filename }

            Label { text: "Key points: "; Layout.alignment: Qt.AlignRight }
            Label { text: Number(mapInfo.keypointCount).toLocaleString(Qt.locale('fi_FI'), 'f', 0) }

            Label { text: "Key frames: "; Layout.alignment: Qt.AlignRight }
            Label { text: Number(mapInfo.keyframeCount).toLocaleString(Qt.locale('fi_FI'), 'f', 0) }
        }
    }
}
