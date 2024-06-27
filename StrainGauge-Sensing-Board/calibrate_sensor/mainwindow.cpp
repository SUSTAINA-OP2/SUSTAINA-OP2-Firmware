#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "foot_sensor_save_manager.h"
#include <QDebug>
#include <Eigen/QR>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow), init_flag_(false)
{
    ui->setupUi(this);
    timer = new QTimer(this);

    timer->setInterval(12.5);

    setup();
    connect(timer, &QTimer::timeout, this, &MainWindow::update);
    connect(ui->calibration_start_pushButton, SIGNAL(clicked()), this, SLOT(calibrationStartPushButtonCliced()));
    connect(ui->offset_pushButton, SIGNAL(clicked()), this, SLOT(pushCalibrateButton()));
    connect(ui->target_foot_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(changeTargetFootBackgroundColor(int)));
    connect(ui->target_sensor_position_comboBox, SIGNAL(activated(int)), this, SLOT(changeTargetSensorPositionBackgroundColor(int)));

    connect(ui->init_pushButton, SIGNAL(clicked()), this, SLOT(initSetup()));
    connect(ui->pushButtonStartCalculation,SIGNAL(clicked()),this,SLOT(onPushStartCalculationButton()));
    connect(ui->pushButtonOutputFIle,SIGNAL(clicked()),this,SLOT(onPushOutputFileButton()));

    timer->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initSetup()
{
    std::call_once(right_foot_init_flag_, [this](){this->initRightFootHandler();});
    std::call_once(left_foot_init_flag_, [this](){this->initLeftFootHandler();});
    /*
    initRightFootHandler();
    initLeftFootHandler();
    std::call_once(right_foot_init_flag_, &MainWindow::initRightFootHandler());
    std::call_once(left_foot_init_flag_, &MainWindow::initLeftFootHandler());
    */

    init_flag_ = true;
}

void MainWindow::setup()
{
    //QString color_str = ui->target_foot_comboBox->palette().color(QWidget::backgroundRole());
    //QColor background_color = ui->target_foot_comboBox->palette().color(QWidget::backgroundRole());
    default_background_color_ = ui->target_foot_comboBox->palette().color(QWidget::backgroundRole());
    qDebug() << default_background_color_.name();

    right_sensor_position_value_list_.append(ui->right_foot_right_front_value_label);
    right_sensor_position_value_list_.append(ui->right_foot_right_rear_value_label);
    right_sensor_position_value_list_.append(ui->right_foot_left_rear_value_label);
    right_sensor_position_value_list_.append(ui->right_foot_left_front_value_label);

    left_sensor_position_value_list_.append(ui->left_foot_right_front_value_label);
    left_sensor_position_value_list_.append(ui->left_foot_right_rear_value_label);
    left_sensor_position_value_list_.append(ui->left_foot_left_rear_value_label);
    left_sensor_position_value_list_.append(ui->left_foot_left_front_value_label);

    right_sensor_position_title_list_.append(ui->right_foot_right_front_title_label);
    right_sensor_position_title_list_.append(ui->right_foot_right_rear_title_label);
    right_sensor_position_title_list_.append(ui->right_foot_left_rear_title_label);
    right_sensor_position_title_list_.append(ui->right_foot_left_front_title_label);

    left_sensor_position_title_list_.append(ui->left_foot_right_front_title_label);
    left_sensor_position_title_list_.append(ui->left_foot_right_rear_title_label);
    left_sensor_position_title_list_.append(ui->left_foot_left_rear_title_label);
    left_sensor_position_title_list_.append(ui->left_foot_left_front_title_label);

    for (auto target_foot_value : right_sensor_position_value_list_)
    {
        target_foot_value->setStyleSheet("background:#ffffff");
    }

    for (auto target_foot_value : left_sensor_position_value_list_)
    {
        target_foot_value->setStyleSheet("background:#ffffff");
    }

    changeTargetFootBackgroundColor();
    changeTargetSensorPositionBackgroundColor(ui->target_sensor_position_comboBox->currentIndex());
}

void MainWindow::initRightFootHandler()
{
    qDebug() << "Initialize RightFootHandler...";
    right_foot_handler_.emplace(TargetFoot::Right);

}

void MainWindow::initLeftFootHandler()
{
    qDebug() << "Initialize LeftFootHandler...";
    left_foot_handler_.emplace(TargetFoot::Left);
}

bool MainWindow::checkCurrentTargetFoot(const QString &check_contains_str)
{
    QString str = ui->target_foot_comboBox->currentText();
    //bool is_target_foot_right = str.contains(check_contains_str, Qt::CaseInsensitive);
    return str.contains(check_contains_str, Qt::CaseInsensitive);
}

void MainWindow::changeTargetFootBackgroundColor(int idx)
{
    //QString str = ui->target_foot_comboBox->currentText();
    bool is_target_foot_right = checkCurrentTargetFoot("right");
    qDebug() << "activate " << idx ;

    if(is_target_foot_right == true){
        ui->right_foot_title_label->setStyleSheet("background:#ffff00");
        ui->left_foot_title_label->setStyleSheet("background:" + default_background_color_.name());
    }else{
        ui->right_foot_title_label->setStyleSheet("background:" + default_background_color_.name());
        ui->left_foot_title_label->setStyleSheet("background:#ffff00");
    }
    changeTargetSensorPositionBackgroundColor(ui->target_sensor_position_comboBox->currentIndex(), true);
}

void MainWindow::changeTargetSensorPositionBackgroundColor(int idx, const bool &change_target_foot)
{
    static int prev_idx_ = -1;
    qDebug() << "arg" << idx ;
    qDebug() << "pre" << prev_idx_ ;
    bool is_target_foot_right = checkCurrentTargetFoot("right");
    QList<QLabel *> target_foot_title_label;
    if(is_target_foot_right == true){
        target_foot_title_label = right_sensor_position_title_list_; 
        if(change_target_foot == true){
            left_sensor_position_title_list_.at(idx)->setStyleSheet("background:" + default_background_color_.name());

        }
    }else{
        target_foot_title_label = left_sensor_position_title_list_; 
        if(change_target_foot == true){
            right_sensor_position_title_list_.at(idx)->setStyleSheet("background:" + default_background_color_.name());
        }

    }


    target_foot_title_label.at(idx)->setStyleSheet("background:#ffff00");
    if(prev_idx_ >= 0 && !(idx == prev_idx_)){
        target_foot_title_label.at(prev_idx_)->setStyleSheet("background:" + default_background_color_.name());
    }

    prev_idx_ = idx;

}

void MainWindow::calibrationStartPushButtonCliced()
{
    //int weight = ui->weight_select_comboBox->currentData().toInt();
    bool change_string_to_int = false;
    const uint weight = ui->weight_select_comboBox->currentText().toUInt(&change_string_to_int);
    const int times =  ui->time_select_spinBox->value();
    if(change_string_to_int == false)
    {
        qDebug() << "\033[41mError:: Invalid weight selected!!!!!!!!!!!!!!!!!!!!!\033[0m";
        return;
    }
    else
    {
        qDebug() << "\033[42m[Start calibrate] Weight is [ " << weight << " ] " << "times is [ " << times << " ]\033[0m";
    }
    TargetFoot target = TargetFoot::Right;
    if(checkCurrentTargetFoot("left")){
        target = TargetFoot::Left;
    }
    auto data = getAverageSensorData(target, times);
    auto target_sensor_position = static_cast<SensorChannel>(ui->target_sensor_position_comboBox->currentIndex());
    if(data)
    {
        if(target == TargetFoot::Right){
            right_measured_data_[target_sensor_position].target_weight = weight;
            auto ave_data = data.value().second.getSensorDataArray(target)[static_cast<size_t>(target_sensor_position)];
            right_measured_data_[target_sensor_position].measured_average_data.insert_or_assign(weight, ave_data);
            qDebug() << "[Start calibrate] Measured data is " << ave_data;
        }
        else
        {
            left_measured_data_[target_sensor_position].target_weight = weight;
            auto ave_data = data.value().second.getSensorDataArray(target)[static_cast<size_t>(target_sensor_position)];
            left_measured_data_[target_sensor_position].measured_average_data.insert_or_assign(weight, ave_data);
            qDebug() << "[Start calibrate] Measured data is " << ave_data;
        }
        qDebug() << "[Start calibrate] Sensor measurement \033[42mSUCCESS\033[0m";
        qDebug() << "[Start calibrate] Write data is " << data.value().second.getSensorDataArray(target)[static_cast<size_t>(target_sensor_position)];
    }
    else{
        qDebug() << "[Start calibrate] Sensor measurement \033[41mERROR!!!!!\033[0m";
    }
}

struct net_and_measured_data
{
    double net_weight;
    double measured_weight;
};

//  scale offset
std::pair<double,double> calcScaleFromMeasuredData(std::vector<net_and_measured_data> data)
{
    using namespace Eigen;
    auto comp = [](const net_and_measured_data& a, const net_and_measured_data& b) { return a.net_weight < b.net_weight; };
    // Sort the data vector in ascending order based on net_weight
    std::sort(data.begin(), data.end(), comp);

    MatrixXd X = MatrixXd::Ones(data.size(), 2);
    size_t index = 0;
    for(const auto& d : data )
    {
        X(index, 0) = d.net_weight;
        index++;
    }
    MatrixXd Y = MatrixXd::Ones(data.size(), 1);
    index = 0;
    for(const auto& d : data )
    {
        Y(index, 0) = d.measured_weight;
        index++;
    }
    // JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    //  // 最小 2 乗解
    // x = svd.solve(b);
    MatrixXd Ans = MatrixXd::Ones(2, 1);
    MatrixXd X_inv = X.completeOrthogonalDecomposition().pseudoInverse();
    Ans = X_inv * Y;
    std::cout << "[calcScaleFromMeasuredData] Least squares solution: scale, offset \n" << Ans.transpose() << std::endl;
    return {1. / Ans(0, 0), Ans(1, 0)};
}

void MainWindow::onPushStartCalculationButton()
{
    std::vector<net_and_measured_data> calc_data;
    calculated_data_.scale.clear();
    calculated_data_.offset.clear();

    if(checkCurrentTargetFoot("right"))
    {
        qDebug() << "------- right foot data-------";
        for (auto [Ch, value] : right_measured_data_)
        {
            qDebug() << "Ch" << static_cast<size_t>(Ch);
            calc_data.clear();
            for (auto [correct_weight, measured_weight] : value.measured_average_data)
            {
                qDebug() << "correct weight" << (correct_weight) << "value" << measured_weight;
                calc_data.push_back({correct_weight, measured_weight});
            }
            auto [scale, offset] = calcScaleFromMeasuredData(calc_data);
            calculated_data_.scale.insert_or_assign(Ch, scale);
            calculated_data_.offset.insert_or_assign(Ch, offset);
        }
        calculated_data_.target_foot = TargetFoot::Right;
        writeCalibrateDataToYAML(calculated_data_);
        if((right_measured_data_.size() != 4) or (calculated_data_.scale.size() != 4))
        {
            qDebug() << "\033[41mError:: Invalid measured data size\033[0m";
            return;
        }
    }
    else if(checkCurrentTargetFoot("left"))
    {
        qDebug() << "------- left foot data-------";
        for (auto [Ch, value] : left_measured_data_)
        {
            qDebug() << "Ch" << static_cast<size_t>(Ch);
            calc_data.clear();
            for (auto [correct_weight, measured_weight] : value.measured_average_data)
            {
                qDebug() << "correct weight" << (correct_weight) << "value" << measured_weight;
                calc_data.push_back({correct_weight, measured_weight});
            }
            auto [scale, offset] = calcScaleFromMeasuredData(calc_data);
            calculated_data_.scale.insert_or_assign(Ch, scale);
            calculated_data_.offset.insert_or_assign(Ch, offset);
        }
        calculated_data_.target_foot = TargetFoot::Left;
        writeCalibrateDataToYAML(calculated_data_);
        if((right_measured_data_.size() != 4) or (calculated_data_.scale.size() != 4))
        {
            qDebug() << "\033[41mError:: Invalid measured data size\033[0m";
            return;
        }
    }
    else
    {
        qDebug() << "\033[41mError:: Invalid target foot\033[0m";
        return;
    
    }
}

void MainWindow::onPushOutputFileButton()
{
    std::string target_foot = "";
    std::vector<int32_t> offset;
    std::vector<int32_t> scale;
    if( (calculated_data_.target_foot == TargetFoot::None) or (calculated_data_.offset.size() != 4) or (calculated_data_.scale.size() != 4))
    {
        qDebug() << "\033[41mBefore output file, please complete calculation\033[0m";
        return ;
    }
    if (calculated_data_.target_foot == TargetFoot::None)
    {
        qDebug() << "\033[41mBefore output file, please start calculation\033[0m";
        return;
    }
    else if (calculated_data_.target_foot == TargetFoot::Right)
    {
        target_foot = "right";
        offset.push_back(right_foot_calibrate_data_.getOffset(SensorChannel::Ch1));
        offset.push_back(right_foot_calibrate_data_.getOffset(SensorChannel::Ch2));
        offset.push_back(right_foot_calibrate_data_.getOffset(SensorChannel::Ch3));
        offset.push_back(right_foot_calibrate_data_.getOffset(SensorChannel::Ch4));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch1));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch2));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch3));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch4));
        qDebug() << "[Output File] right foot data writing...";
    }
    else if (calculated_data_.target_foot == TargetFoot::Left)
    {
        target_foot = "left";
        offset.push_back(left_foot_calibrate_data_.getOffset(SensorChannel::Ch1));
        offset.push_back(left_foot_calibrate_data_.getOffset(SensorChannel::Ch2));
        offset.push_back(left_foot_calibrate_data_.getOffset(SensorChannel::Ch3));
        offset.push_back(left_foot_calibrate_data_.getOffset(SensorChannel::Ch4));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch1));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch2));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch3));
        scale.push_back(calculated_data_.scale.at(SensorChannel::Ch4));
        qDebug() << "[Output File] left foot data writing...";
    }
    else
    {
        qDebug() << "\033[41mError:: Invalid target foot\033[0m";
        return;
    }
    FootSensorSaveManager save_manager(target_foot);
    int times = ui->offset_times_spinBox->value();
    save_manager.saveOffsetData(offset, times , 0);
    save_manager.saveScaleData(scale,times,0,0);
    calculated_data_.target_foot = TargetFoot::None;
    calculated_data_.offset.clear();
    calculated_data_.scale.clear();
    return;
}

void MainWindow::update()
{
    //getSensorData();
    //std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    auto right_foot_sensor_data = getOneSensorData(TargetFoot::Right);
    auto left_foot_sensor_data = getOneSensorData(TargetFoot::Left);

    if(right_foot_sensor_data){
        auto sensor_data = right_foot_sensor_data.value();
        right_foot_calibrate_data_.setFootSensorData(sensor_data);
        auto result_arr = right_foot_calibrate_data_.getCalibrateSensorDataArray(TargetFoot::Right);
        FootSensorData<int32_t> set_data(result_arr[0], result_arr[1], result_arr[2], result_arr[3]);
        displaySensorData(set_data, TargetFoot::Right);        
    }

    if(left_foot_sensor_data){
        auto sensor_data = left_foot_sensor_data.value();
        left_foot_calibrate_data_.setFootSensorData(sensor_data);
        auto result_arr = left_foot_calibrate_data_.getCalibrateSensorDataArray(TargetFoot::Left);
        FootSensorData<int32_t> set_data(result_arr[0], result_arr[1], result_arr[2], result_arr[3]);
        displaySensorData(set_data, TargetFoot::Left);   
    }
}

std::optional<FootSensorData<int32_t>> MainWindow::getOneSensorData(const TargetFoot &target_foot)
{
    if(init_flag_ == false) return std::nullopt;

    FootSensorHandler *target_foot_handler = nullptr;
    if(target_foot == TargetFoot::Right){
        if(right_foot_handler_){
            target_foot_handler = &right_foot_handler_.value();
        }
    }else{
        if(left_foot_handler_){
            target_foot_handler = &left_foot_handler_.value();
        }
    }

    if(target_foot_handler == nullptr){
        return std::nullopt;
    }
    return target_foot_handler->getOneSensorData();
}


std::optional<std::pair<std::vector<FootSensorData<int32_t>>, FootSensorData<int32_t>>> MainWindow::getAverageSensorData(const TargetFoot &target_foot, const int32_t &times)
{
    FootSensorHandler *target_foot_handler = nullptr;
    if(target_foot == TargetFoot::Right){
        if(right_foot_handler_){
            target_foot_handler = &right_foot_handler_.value();
        }
    }else{
        if(left_foot_handler_){
            target_foot_handler = &left_foot_handler_.value();
        }
    }

    if(target_foot_handler != nullptr){
        return target_foot_handler->getTimesAverageSensorData(times);
    }else{
        return std::nullopt;
    }
    /*
    auto [sensor_data_vec, average_data] = target_foot_handler->getTimesAverageSensorData(times);
    return {sensor_data_vec, average_data};
    */
}

void MainWindow::displaySensorData(const FootSensorData<int32_t> &data, const TargetFoot &target_foot)
{
    
    std::array<int32_t, 4> each_data = data.getSensorDataArray(target_foot);
    QList<QLabel *> target_foot_value_list;
    if(target_foot == TargetFoot::Right)
    {
        target_foot_value_list = right_sensor_position_value_list_; 
    }else{
        target_foot_value_list = left_sensor_position_value_list_; 
    }
    for (std::size_t i=0; auto target_foot_value : target_foot_value_list)
    {
        target_foot_value->setNum(each_data.at(i));

        i++;
    }
}

void MainWindow::pushCalibrateButton()
{
    calibrateOffset();
}

void MainWindow::setSensorData(const FootSensorData<int32_t> &sensor_data, const TargetFoot &target_foot)
{
    if(target_foot == TargetFoot::Right){
        right_foot_calibrate_data_.foot_sensor = sensor_data;

    }else if(target_foot == TargetFoot::Left){
        left_foot_calibrate_data_.foot_sensor = sensor_data;
    }
}

void MainWindow::setOffsetData(const FootSensorData<int32_t> &offset_data, const TargetFoot &target_foot)
{
    if(target_foot == TargetFoot::Right){
        right_foot_calibrate_data_.setOffset(offset_data);
    }else if(target_foot == TargetFoot::Left){
        left_foot_calibrate_data_.setOffset(offset_data);
    }
}

void MainWindow::setScaleData(const double &scale_data, const TargetFoot &target_foot, const SensorChannel &target_sensor_channel)
{
    if(target_foot == TargetFoot::Right){
        right_foot_calibrate_data_.setScale(target_sensor_channel, scale_data);
    }else if(target_foot == TargetFoot::Left){
        left_foot_calibrate_data_.setScale(target_sensor_channel, scale_data);
    }
}

void MainWindow::calibrateOffset()
{
    int times = ui->offset_times_spinBox->value();
    qDebug() << "calibrate offset" << times;
    auto right_sensor_data = getAverageSensorData(TargetFoot::Right, times);
    auto left_sensor_data = getAverageSensorData(TargetFoot::Left, times);

    if(right_sensor_data){
        auto [sensor_data_vec, each_average_data] = right_sensor_data.value();
        //right_foot_calibrate_data_.setOffset(each_average_data);
        setOffsetData(each_average_data, TargetFoot::Right);
        qDebug() << "Set right foot offset";
        qDebug() <<  each_average_data.sensor_data_ch1;
        qDebug() <<  each_average_data.sensor_data_ch2;
        qDebug() <<  each_average_data.sensor_data_ch3;
        qDebug() <<  each_average_data.sensor_data_ch4;
    }

    if(left_sensor_data){
        auto [sensor_data_vec, each_average_data] = left_sensor_data.value();
        setOffsetData(each_average_data, TargetFoot::Left);
        qDebug() << "Set left foot offset";
        qDebug() <<  each_average_data.sensor_data_ch1;
        qDebug() <<  each_average_data.sensor_data_ch2;
        qDebug() <<  each_average_data.sensor_data_ch3;
        qDebug() <<  each_average_data.sensor_data_ch4;
        //left_foot_calibrate_data_.setOffset(each_average_data);
    }

    /*
    if(right_foot_handler_){
        //auto [each_data_all, each_data_average] = right_foot_handler_->getTimesAverageSensorData(times);
        auto [each_data_vec, each_average_data] = getAverageSensorData(TargetFoot::Right, times);
        right_foot_calibrate_data_.setOffset(each_average_data);
    }
    if(left_foot_handler_){
        //auto [each_data_all, each_data_average] = left_foot_handler_->getTimesAverageSensorData(times);
        auto [each_data_vec, each_average_data] = getAverageSensorData(TargetFoot::Left, times);
        left_foot_calibrate_data_.setOffset(each_average_data);
    }
    */
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    qDebug() << "after calibrate offseti" << times;
}

void MainWindow::writeCalibrateDataToYAML(const CalculatedResult &calibrate_data)
{
    std::string target_foot_file_name = "right_foot_sensor_bias.yml";
    //const std::array<std::string, 4> sensor_position_name = {"scale_right_front", "scale_right_rear", "scale_left_front", "scale_left_rear"};

    const std::unordered_map<SensorChannel, std::string> sensor_position_name{
        {SensorChannel::Ch1, "scale_right_front"},
        {SensorChannel::Ch2, "scale_right_rear"},
        {SensorChannel::Ch4, "scale_left_front"},
        {SensorChannel::Ch3, "scale_left_rear"}};

    if(calibrate_data.target_foot == TargetFoot::Left){
        target_foot_file_name = "left_foot_sensor_bias.yml";
    }
    std::unordered_map<std::string, double> scale{
        {"scale_right_front", 1},
        {"scale_right_rear", 1},
        {"scale_left_front", 1},
        {"scale_left_rear", 1}};

    YAML::Node node;

    for ( auto [ch, scale_data] : calibrate_data.scale)
    {
        scale[sensor_position_name.at(ch)] = scale_data;
    }

    node["scale_right_front"] = scale.at("scale_right_front");
    node["scale_right_rear"] = scale.at("scale_right_rear");
    node["scale_left_front"] = scale.at("scale_left_front");
    node["scale_left_rear"] = scale.at("scale_left_rear");

    node["offset_right_front"] = 0;
    node["offset_right_rear"] = 0;
    node["offset_left_front"] = 0;
    node["offset_left_rear"] = 0;

    std::ofstream ofs(target_foot_file_name);
    ofs << node;
}

