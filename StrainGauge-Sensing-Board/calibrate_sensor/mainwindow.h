#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QString>
#include <QLabel>

#include <optional>
#include <mutex>
#include <unordered_map>

#include <foot_sensor_handler.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


struct MeasuredDataForCalibration
{
    //key:正確な重量、value:計測重量
    std::unordered_map<int32_t,int32_t> measured_average_data;
    int32_t target_weight;
};

struct CalculatedResult
{
    TargetFoot target_foot;
    std::unordered_map<SensorChannel,int32_t> offset; // オフセット(切片)
    std::unordered_map<SensorChannel,double> scale;   // スケール(傾き)
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    std::optional<FootSensorData<int32_t>> getOneSensorData(const TargetFoot &target_foot);

private slots:
    void calibrationStartPushButtonCliced();
    void calibrateOffset();
    void changeTargetFootBackgroundColor(int idx = -1);
    void changeTargetSensorPositionBackgroundColor(int idx = -1, const bool &change_target_foot=false);

    void initSetup();
    void pushCalibrateButton();
    void onPushStartCalculationButton();
    void onPushOutputFileButton();

private:

    void setup();
    void update();
    bool checkCurrentTargetFoot(const QString &check_contains_str);
    void displaySensorData(const FootSensorData<int32_t>&, const TargetFoot &);

    std::optional<std::pair<std::vector<FootSensorData<int32_t>>, FootSensorData<int32_t>>> getAverageSensorData(const TargetFoot &, const int32_t &);

    void setSensorData(const FootSensorData<int32_t> &, const TargetFoot &);
    void setOffsetData(const FootSensorData<int32_t> &, const TargetFoot &);
    void setScaleData(const double &, const TargetFoot &, const SensorChannel &);

    void initRightFootHandler();
    void initLeftFootHandler();

    void writeCalibrateDataToYAML(const CalculatedResult &calibrate_data);
    void readBiasFile(const TargetFoot &target_foot);

    Ui::MainWindow *ui;
    QTimer *timer;
    QColor default_background_color_;

    QList<QLabel *> right_sensor_position_value_list_;
    QList<QLabel *> left_sensor_position_value_list_;

    QList<QLabel *> right_sensor_position_title_list_;
    QList<QLabel *> left_sensor_position_title_list_;

    std::optional<FootSensorHandler> right_foot_handler_;
    std::optional<FootSensorHandler> left_foot_handler_;

    FootSensorCalibrateData right_foot_calibrate_data_;
    FootSensorCalibrateData left_foot_calibrate_data_;

    std::unordered_map<SensorChannel,MeasuredDataForCalibration> right_measured_data_;
    std::unordered_map<SensorChannel,MeasuredDataForCalibration> left_measured_data_;

    CalculatedResult calculated_data_;

    std::once_flag right_foot_init_flag_;
    std::once_flag left_foot_init_flag_;

    bool init_flag_;
};

#endif // MAINWINDOW_H

