#ifndef HMI_GUI_HPP
#define HMI_GUI_HPP

#include <QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QPushButton>

class HmiGui : public QMainWindow {
  Q_OBJECT
public:
    explicit HmiGui(QWidget *parent = nullptr);
    ~HmiGui();

public slots:
    /// Slot to update the GUI based on the received blackboard status.
    /// The parameters are interpreted as:
    /// - execution_resumed: current execution state,
    /// - stop_execution: if the stop has been triggered,
    /// - abort_mission: if an abort is active.
    void updateStatus(bool execution_resumed, bool stop_execution, bool abort_mission);

    // Slots that are called when the buttons are pressed.
    void onStartClicked();
    void onStopClicked();
    void onResetClicked();

    // Slots for TCP server events.
    void onNewConnection();
    void onSocketDisconnected();

signals:
    /// Signals that request the corresponding ROS2 service call.
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

    // Stores the last status as a JSON string to send to TCP clients.
    QString        lastStatusJson_;
};

#endif // HMI_GUI_HPP
