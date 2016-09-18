#include <QtGui>
#include <QPushButton>
#include <QLineEdit>
#include <QMainWindow>

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
    void outlogtext(QString ver);

private slots:
    void outlog();
    void outlogRos();
    void on_pushButton_24_clicked();
    void on_pushButtonRos_clicked();
    void on_pushButtonKill_clicked();

private:
    QPushButton* pushButton_24;
    QPushButton* pushButtonRos_;
    QPushButton* pushButtonKill_;
    QLineEdit* lineEdit_4;
    QLineEdit* lineEditRos_;
    QLineEdit* lineEditKill_;
    QProcess* helloProcess_;
    QProcess* rosProcess_;

    int rosLaunchPid_;
};


