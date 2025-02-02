#include "manymove_hmi/hmi_gui.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDebug>

HmiGui::HmiGui(QWidget *parent)
    : QMainWindow(parent), clientSocket_(nullptr)
{
    // Create central widget and layout
    centralWidget_ = new QWidget(this);
    setCentralWidget(centralWidget_);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget_);

    // Create the three buttons.
    startButton_ = new QPushButton("START", this);
    stopButton_ = new QPushButton("STOP", this);
    resetButton_ = new QPushButton("RESET", this);

    // Set object names so that the style sheet rules for #startButton, etc., apply.
    startButton_->setObjectName("startButton");
    stopButton_->setObjectName("stopButton");
    resetButton_->setObjectName("resetButton");

    // Set initial states (assume that with stop_execution true, START and RESET are enabled).
    startButton_->setEnabled(true);
    resetButton_->setEnabled(true);
    stopButton_->setEnabled(false);

    // Connect the button signals to our internal slots.
    connect(startButton_, &QPushButton::clicked, this, &HmiGui::onStartClicked);
    connect(stopButton_, &QPushButton::clicked, this, &HmiGui::onStopClicked);
    connect(resetButton_, &QPushButton::clicked, this, &HmiGui::onResetClicked);

    // Layout the buttons.
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(startButton_);
    buttonLayout->addWidget(stopButton_);
    buttonLayout->addWidget(resetButton_);
    mainLayout->addLayout(buttonLayout);

    // Set up a TCP server listening on port 5000.
    tcpServer_ = new QTcpServer(this);
    connect(tcpServer_, &QTcpServer::newConnection, this, &HmiGui::onNewConnection);
    if (!tcpServer_->listen(QHostAddress::Any, 5000))
    {
        qWarning() << "TCP Server could not start!";
    }
    else
    {
        qDebug() << "TCP Server listening on port 5000";
    }
}

HmiGui::~HmiGui()
{
    if (clientSocket_)
    {
        clientSocket_->disconnectFromHost();
    }
}

void HmiGui::updateStatus(bool execution_resumed, bool stop_execution, bool abort_mission)
{
    // Update button states:
    // - STOP button is enabled only if stop_execution is false.
    // - START and RESET are enabled only if stop_execution is true.
    if (stop_execution)
    {
        startButton_->setEnabled(true);
        resetButton_->setEnabled(true);
        stopButton_->setEnabled(false);
    }
    else
    {
        startButton_->setEnabled(false);
        resetButton_->setEnabled(false);
        stopButton_->setEnabled(true);
    }

    // Build a JSON string representing the current status.
    lastStatusJson_ = QString("{\"execution_resumed\": %1, \"stop_execution\": %2, \"abort_mission\": %3}")
                          .arg(execution_resumed ? "true" : "false")
                          .arg(stop_execution ? "true" : "false")
                          .arg(abort_mission ? "true" : "false");

    // If a TCP client is connected, send the updated status.
    if (clientSocket_ && clientSocket_->state() == QAbstractSocket::ConnectedState)
    {
        clientSocket_->write(lastStatusJson_.toUtf8());
        clientSocket_->flush();
    }
}

void HmiGui::onStartClicked()
{
    emit startExecutionRequested();
}

void HmiGui::onStopClicked()
{
    emit stopExecutionRequested();
}

void HmiGui::onResetClicked()
{
    emit resetProgramRequested();
}

void HmiGui::onNewConnection()
{
    clientSocket_ = tcpServer_->nextPendingConnection();
    connect(clientSocket_, &QTcpSocket::disconnected, this, &HmiGui::onSocketDisconnected);
    qDebug() << "New TCP client connected.";
    // Optionally, send the current status immediately.
    if (!lastStatusJson_.isEmpty())
    {
        clientSocket_->write(lastStatusJson_.toUtf8());
        clientSocket_->flush();
    }
}

void HmiGui::onSocketDisconnected()
{
    qDebug() << "TCP client disconnected.";
    clientSocket_ = nullptr;
}
