#ifndef HMI_GUI_HPP
#define HMI_GUI_HPP

#include <QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QPushButton>
#include <QLabel>

class HmiGui : public QMainWindow {
  Q_OBJECT
public:
    explicit HmiGui(QWidget *parent = nullptr);
    ~HmiGui();

public slots:
    /// Update the GUI based on the current blackboard status.
    /// Now also includes collision_detected.
    void updateStatus(bool execution_resumed, bool stop_execution, bool abort_mission, bool collision_detected);

    // Slots called when buttons are pressed.
    void onStartClicked();
    void onStopClicked();
    void onResetClicked();

    // Slots for TCP server events.
    void onNewConnection();
    void onSocketDisconnected();

signals:
    /// Signals requesting the corresponding ROS2 service calls.
    void startExecutionRequested();
    void stopExecutionRequested();
    void resetProgramRequested();

private:
    QWidget       *centralWidget_;
    QPushButton   *startButton_;
    QPushButton   *stopButton_;
    QPushButton   *resetButton_;

    QTcpServer    *tcpServer_;
    QTcpSocket    *clientSocket_;

    // LED-like indicator for collision_detected.
    QLabel        *ledIndicator_;

    // Stores the last status as a JSON string for TCP clients.
    QString        lastStatusJson_;
};

#endif // HMI_GUI_HPP
