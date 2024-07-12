#include "IMUCalibrationWidget.h"

#include <cmath>

#include <string>
#include <iostream>
#include <libgen.h>

#include <ros/ros.h>

// #include <imu_tk/io_utils.h>
#include <imu_tk/calibration.h>
#include <imu_tk/filters.h>
#include <imu_tk/integration.h>
#include <imu_tk/visualization.h>
#include <locale.h>

#include <ros/master.h>
#include <ros/package.h>

#include <dirent.h>

#include <boost/filesystem.hpp>


imu_tk::MultiPosCalibration mp_calib;
const std::string IMU_SUB = "IMU_SUB";

IMUCalibrationWidget::IMUCalibrationWidget(QWidget *parent)
	: BaseWidget(parent)
{
	setupUi(this);
	connect(recordDataChooseFileButton, &QPushButton::clicked, this, [this]
			{ IMUCalibrationWidget::chooseFileButton_clicked(pathOfRecordData, QFileDialog::AcceptSave); });
	connect(calibrateChooseFileButton, &QPushButton::clicked, this, [this]
			{ IMUCalibrationWidget::chooseFileButton_clicked(pathOfCalibrateData, QFileDialog::AcceptOpen); });
	connect(calibratedParameterChooseDirectoryButton, &QPushButton::clicked, this, [this]
			{ IMUCalibrationWidget::chooseDirectoryButton_clicked(pathOfCalibratedParameter, QFileDialog::AcceptOpen); });

	connect(&timerOfProgressBar, &QTimer::timeout, this, &IMUCalibrationWidget::updateProgressBar);
	connect(&timerOfProgressText, &QTimer::timeout, this, &IMUCalibrationWidget::updateProgressText);
	connect(&timerOfInitializationPeriod, &QTimer::timeout, this, &IMUCalibrationWidget::startProgressTextTimer);
}

IMUCalibrationWidget::~IMUCalibrationWidget()
{
	// TODO Auto-generated destructor stub
}

QWidget *IMUCalibrationWidget::createInstance()
{
	return new IMUCalibrationWidget;
}

void IMUCalibrationWidget::init()
{
	//addSubscriber(IMU_SUB, "/GETjag/imu/data", 2, &IMUCalibrationWidget::imuCallback, this);
}

void IMUCalibrationWidget::imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
	if (readData)
	{
		auto timeStamp = ros::Time::now() - startTime;
		QTextStream out(&dataFile);
		out.setRealNumberNotation(QTextStream::ScientificNotation);
		out.setRealNumberPrecision(8);
		out << timeStamp.toSec() << " " << imuMsg->linear_acceleration.x << " " << imuMsg->linear_acceleration.y << " " << imuMsg->linear_acceleration.z 
								 << " " << imuMsg->angular_velocity.x << " " << imuMsg->angular_velocity.y << " " << imuMsg->angular_velocity.z << "\n";
	}
}

// When the select button is pressed, the topic that user had entered will be selected as subscriber
void IMUCalibrationWidget::on_imuSelectionButton_clicked()
{		
	imuTopic = imuTopicsField->text();

	//check whether entered IMU topic is available
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{

		const ros::master::TopicInfo& info = *it;
		std::cout <<info.name << std::endl;
		if(info.name == imuTopic.toStdString())
		{
			//add subscriber if correct topic is entered
			addSubscriber(IMU_SUB, imuTopic.toStdString(), 2, &IMUCalibrationWidget::imuCallback, this);
		}
	}
}

// When you press the start button for this function, data recording will begin.
void IMUCalibrationWidget::on_startRecordButton_clicked()
{
	if (started)
	{
		started = false;
		startRecordButton->setText("Start");
		bool ifInit = (timerOfInitializationPeriod.isActive()) ? true : false;
		timerOfProgressBar.stop();
		timerOfInitializationPeriod.stop();
		timerOfProgressText.stop();
		readData = false;
		int ret = createMessageBox("Are you sure you want to stop recording IMU?", "This will stop recording and go back to main window.", true);
		switch (ret)
		{
		case QMessageBox::Yes:
			dataFile.close();
			returnToMainWindow();
			break;
		case QMessageBox::No:
			if (ifInit)
			{
				timerOfInitializationPeriod.start();
			}
			else
			{
				timerOfProgressText.start();
			}
			readData = true;
			timerOfProgressBar.start();
			break;
		}
	}
	else
	{
		started = true;
		startRecordButton->setText("Stop");
		if (supportedFileTypeCheck(pathOfRecordData->text()))
		{
			dataFile.setFileName(pathOfRecordData->text());
			if (!dataFile.open(QIODevice::WriteOnly | QIODevice::Text))
			{
				QMessageBox messageBox;
				messageBox.critical(0, "Error", QString("There is no file called '%1'").arg(pathOfRecordData->text()));
				dataFile.close();
				returnToMainWindow();
			}
			else
			{
				startTime = ros::Time::now();
				recordGroupBox->setEnabled(false);
				calibrateGroupBox->setEnabled(false);
				// recordCancelButton->setEnabled(true);
				readData = true;
				timerOfProgressBar.start(1000);
				progressText->setText(QString("Now, hold IMU static for %1 seconds for initialization!").arg(staticInitializationDuration->value()));
				setIcon("media-playback-stop");
				setAttitudeImage("1","2");
				cycleTime = durationOfStaticIntervals->value() + durationOfMotionIntervals->value();
				timerOfInitializationPeriod.start(staticInitializationDuration->value() * 1000);
				pathOfCalibrateData->setText(pathOfRecordData->text());
			}
		}
	}
}

// To save calibrated dat this function will save file at selected location.
void IMUCalibrationWidget::chooseDirectoryButton_clicked(QLineEdit *path, QFileDialog::AcceptMode mode)
{
	QFileDialog dialog(this);
	dialog.setFileMode(QFileDialog::Directory);
	dialog.setViewMode(QFileDialog::Detail);
	dialog.setAcceptMode(mode);
	QString directory = QFileDialog::getExistingDirectory(this, tr("Select Directory"),
														  path->text(),
														  QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	path->clear();
	path->insert(directory);
}


void IMUCalibrationWidget::chooseFileButton_clicked(QLineEdit *path, QFileDialog::AcceptMode mode)
{
	QFileDialog dialog(this);
	QString fileName;
	dialog.setFileMode(QFileDialog::AnyFile);
	dialog.setViewMode(QFileDialog::Detail);
	dialog.setAcceptMode(mode);
	if (mode == QFileDialog::AcceptSave)
	{
		fileName = QFileDialog::getSaveFileName(this, tr("Save File"), path->text(), tr("Files (*.mat)"));
	}
	else
	{
		fileName = QFileDialog::getOpenFileName(this, tr("Open File"), path->text(), tr("Files (*.mat)"));
	}
	path->clear();
	path->insert(fileName);
}

void IMUCalibrationWidget::updateProgressBar()
{
	float maxValue = staticInitializationDuration->value() + cycleTime * numberOfPoses->value();
	sumOfProgress += 100 / maxValue;
	progressBar->setValue(sumOfProgress);
}

void IMUCalibrationWidget::updateProgressText()
{
	if (counter >= numberOfPoses->value())
	{
		returnToMainWindow();
	}
	else
	{
		if (std::abs(elapsedTime - (counter * (cycleTime) + durationOfMotionIntervals->value()) * 100) < 1)
		{
			progressText->setText(QString("Now, hold the IMU static for %1 seconds!").arg(durationOfStaticIntervals->value()));
			setIcon("media-playback-stop");
			int imageNumber = count;
			int nextImageNumber = count + 1;
			QString  image = QString::number(imageNumber);
			QString  nextImage = QString::number(nextImageNumber);
			setAttitudeImage(image, nextImage);
			counter += 1;
			count += 1;
			if (imageNumber == 12){count = 1;}
		}
		else if (std::abs (elapsedTime - (counter * cycleTime) * 100) < 1)
		{
			progressText->setText("Now, change the position of the IMU!");
			setIcon("media-playback-start");
		}
	}
	timeAttitude->setNum((std::abs(int(elapsedTime - (counter * (cycleTime) + durationOfMotionIntervals->value()) * 100)/100)));
	elapsedTime += 1;
}

template <typename _T>
void IMUCalibrationWidget::readDataFromFile(const QString &fileName,
										 std::vector<imu_tk::TriadData_<_T>> &samples0,
										 std::vector<imu_tk::TriadData_<_T>> &samples1,
										 TimestampUnit unit, DatasetType type)
{
	setlocale(LC_NUMERIC, "C");
	samples0.clear();
	samples1.clear();

	QFile infile(fileName);
	double ts, d[6];

	if (!infile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		printf("Data File is anable Open");
		return;
	}

	QTextStream in(&infile);
	if (infile.isOpen())
	{
		char format[266];
		switch (type)
		{
		case DATASET_COMMA_SEPARATED:
			qsnprintf(format, 266, "%%lf, %%lf, %%lf, %%lf, %%lf, %%lf, %%lf");
			break;
		case DATASET_SPACE_SEPARATED:
		default:
			qsnprintf(format, 266, "%%lf %%lf %%lf %%lf %%lf %%lf %%lf");
			break;
		}

		int l = 0;
		while (!in.atEnd())
		{
			QString line = in.readLine();

			int res = sscanf(line.toStdString().c_str(), format, &ts, &d[0], &d[1], &d[2],
							 &d[3], &d[4], &d[5]);
			if (res != 7)
			{
				std::cout << "readDataFromFile(): error importing data in line " << l << ", exit" << std::endl;
			}
			else
			{
				ts /= unit;
				samples0.push_back(imu_tk::TriadData_<_T>(_T(ts), _T(d[0]), _T(d[1]), _T(d[2])));
				samples1.push_back(imu_tk::TriadData_<_T>(_T(ts), _T(d[3]), _T(d[4]), _T(d[5])));
			}
			l++;
		}
		infile.close();
	}
}

void IMUCalibrationWidget::on_startCalibrateButton_clicked()
{
	dataFile.setFileName(pathOfCalibrateData->text());
	supportedFileTypeCheck(pathOfCalibrateData->text());
	if (!dataFile.exists())
	{
		createMessageBox("The file does not exist!", "Please choose a valid file.", false);
		returnToMainWindow();
	}
	else
	{
		recordGroupBox->setEnabled(false);
		// calibrateCancelButton->setEnabled(true);
		calibrateGroupBox->setEnabled(false);
		std::vector<imu_tk::TriadData> acc_data, gyro_data;

		std::cout << "Importing IMU data from the Matlab matrix file : " << pathOfCalibrateData->text().toStdString() << std::endl;
		readDataFromFile(pathOfCalibrateData->text().toStdString().c_str(), acc_data, gyro_data, TIMESTAMP_UNIT_SEC, DATASET_SPACE_SEPARATED);
		imu_tk::CalibratedTriad init_acc_calib, init_gyro_calib;
		init_acc_calib.setBias(Eigen::Vector3d(0, 0, 0)); // 32768, 32768, 32768
		init_gyro_calib.setScale(Eigen::Vector3d(1.0 / 6258.0, 1.0 / 6258.0, 1.0 / 6258.0));
		// init_acc_calib.setBias(Eigen::Vector3d(0, 0, 0));
		// init_acc_calib.setScale(Eigen::Vector3d(1, 1, 1));
		// init_gyro_calib.setBias(Eigen::Vector3d(0, 0, 0));
		// init_gyro_calib.setScale(Eigen::Vector3d(1, 1, 1));

		// imu_tk::MultiPosCalibration mp_calib;
		mp_calib.setInitStaticIntervalDuration(staticInitializationDuration->value());
		mp_calib.setInitAccCalibration(init_acc_calib);
		mp_calib.setInitGyroCalibration(init_gyro_calib);
		mp_calib.setGravityMagnitude(9.81189);
		mp_calib.enableVerboseOutput(true);
		mp_calib.enableAccUseMeans(true);
		bool result = mp_calib.calibrateAccGyro(acc_data, gyro_data);
		std::cout << "p_calib.calibrateAccGyro: " << result << "\n";
		// mp_calib.getAccCalib().save("test_data/test_imu_acc.calib");
		// mp_calib.getGyroCalib().save("test_data/test_imu_gyro.calib");
		ResultgroupBox->setEnabled(true);
		saveParameterButton->setEnabled(true);
		calibratedParameterChooseDirectoryButton->setEnabled(true);
		QMessageBox::information(this, "Information", "The calibration process is complete. Please select folder where you wish to save the calibration parameter.");
	}
}

void IMUCalibrationWidget::on_saveParameterButton_clicked()
{
	QString ParameterFile = "/CalibrationConfiguration.yaml";
	QFile CalibParamfile(pathOfCalibratedParameter->text() + ParameterFile);
	if (!CalibParamfile.open(QIODevice::ReadWrite | QIODevice::Text))
	{
		return;
	}
	QTextStream out(&CalibParamfile);

	// out.setRealNumberNotation(QTextStream::ScientificNotation);
	// out.setRealNumberPrecision(8);
	out << "# #-------------Accelerometer-----------" << endl;
	
	out << "accel_misalignment:" <<" "<<"["<< "1"  <<","  << mp_calib.getAccCalib().misYZ() <<","<< mp_calib.getAccCalib().misZY() <<"," << endl
        		<< mp_calib.getAccCalib().misXZ() <<","<< "1" <<","<< mp_calib.getAccCalib().misZX() <<"," << endl
        		<< mp_calib.getAccCalib().misXY() <<","<< mp_calib.getAccCalib().misYX() <<","<< "1" <<"]" << endl << endl;
	
	out << "accel_scale:" <<" "<<"["<< mp_calib.getAccCalib().scaleX() <<","<< "0" <<","<< "0" <<"," << endl
        << "0" <<","<< mp_calib.getAccCalib().scaleY() <<","<< "0" <<"," << endl
        << "0" <<","<< "0" <<","<< mp_calib.getAccCalib().scaleZ() <<"]" << endl << endl;
	
	out << "accel_bias:" <<" "<< "[" << mp_calib.getAccCalib().biasX() <<","<< mp_calib.getAccCalib().biasY() <<","<< mp_calib.getAccCalib().biasZ() << "]" << endl << endl;

	out << "# #-------------Gyroscope-----------" << endl;
	out << "gyro_misalignment:" <<" "<<"["<< "1"  <<"," << mp_calib.getGyroCalib().misYZ() <<","<< mp_calib.getGyroCalib().misZY() <<"," << endl
    			<< mp_calib.getGyroCalib().misXZ() <<","<< "1" <<","<< mp_calib.getGyroCalib().misZX() <<"," << endl
        		<< mp_calib.getGyroCalib().misXY() <<","<< mp_calib.getGyroCalib().misYX() <<","<< "1" <<"]" << endl << endl;
	
	out << "gyro_scale:" <<" "<<"["<< mp_calib.getGyroCalib().scaleX() <<","<< "0" <<","<< "0" <<"," << endl
        << "0" <<","<< mp_calib.getGyroCalib().scaleY() <<","<< "0" <<"," << endl
        << "0" <<","<< "0" <<","<< mp_calib.getGyroCalib().scaleZ() <<"]" << endl << endl;
	
	out << "gyro_bias:" <<" "<< "[" << mp_calib.getGyroCalib().biasX() <<","<< mp_calib.getGyroCalib().biasY() <<","<< mp_calib.getGyroCalib().biasZ() << "]" << endl << endl;

	CalibParamfile.close();
	
	int ret = createMessageBox("The calibration parameters are saved..", "Do you want to close the window?", true);
	switch (ret)
	{
	case QMessageBox::Yes:
		dataFile.close();
		returnToMainWindow();
		QApplication::quit();
		break;
	case QMessageBox::No:
		// TODO
		break;
	}
}

bool IMUCalibrationWidget::supportedFileTypeCheck(QString path)
{
	QFileInfo fi(path);
	QString ext = fi.suffix();
	if (ext != "mat")
	{
		createMessageBox("The file type is not supported!", "Please check the extension of the file!", false);
		returnToMainWindow();
		return false;
	}
	return true;
}

void IMUCalibrationWidget::setIcon(QString iconName)
{
	QIcon icon = QIcon::fromTheme(iconName);
	QPixmap pixmap = icon.pixmap(QSize(20, 20));
	iconLabel->setPixmap(pixmap);
	iconLabel->setAlignment(Qt::AlignRight);
}

void IMUCalibrationWidget::setAttitudeImage(QString imageName, QString nextImageName)
{
	char file[] = __FILE__;
	int width = 200;
	int height = 200;
	QString imagePath = dirname(file);
	QPixmap pixmap(imagePath + "/picture/" + imageName + ".png");
	QPixmap newPixmap = pixmap.scaled(width, height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	attitudeImage->setPixmap(newPixmap);

	QPixmap nextImagePixmap(imagePath + "/picture/" + nextImageName + ".png");
	QPixmap newNextImagePixmap = nextImagePixmap.scaled(width, height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
	nextImage->setPixmap(newNextImagePixmap);

	
}

void IMUCalibrationWidget::on_resetCalibrationButton_clicked()
{
	// on_resetImuButton_clicked();
	returnToMainWindow();
	// imuSelect->setCurrentText("Select IMU...");
}

void IMUCalibrationWidget::returnToMainWindow()
{
	sumOfProgress = 0;
	elapsedTime = 0;
	counter = 0;
	count = 1;
	readData = false;
	recordGroupBox->setEnabled(true);
	// recordCancelButton->setEnabled(false);
	calibrateGroupBox->setEnabled(true);
	progressBar->setValue(0);
	// calibrateCancelButton->setEnabled(false);
	ResultgroupBox->setEnabled(false);
	saveParameterButton->setEnabled(false);
	calibratedParameterChooseDirectoryButton->setEnabled(false);
	timerOfProgressBar.stop();
	progressText->setText("");
	startRecordButton->setText("Start");
	pathOfCalibrateData->setText("");
	imuTopicsField->setText("");
	pathOfRecordData->setText("");
	pathOfCalibratedParameter->setText("");
	setIcon("");
	timerOfInitializationPeriod.stop();
	timerOfProgressText.stop();	
}

int IMUCalibrationWidget::createMessageBox(QString text, QString informativeText, bool button)
{
	msgBox.setText(text);
	msgBox.setInformativeText(informativeText);
	msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox.setDefaultButton(QMessageBox::No);
	if (button)
	{
		msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
		msgBox.setDefaultButton(QMessageBox::No);
	}
	int ret = msgBox.exec();
	return ret;
}

void IMUCalibrationWidget::startProgressTextTimer()
{
	timerOfInitializationPeriod.stop();
	timerOfProgressText.start(10);
}
