#ifndef SRC_IMU_CALIBRATER_GUI_CALIBRATIONWIDGET_H_
#define SRC_IMU_CALIBRATER_GUI_CALIBRATIONWIDGET_H_

#include <QtCore/QtPlugin>
#include <QtWidgets/QWidget>

#include <core/BaseWidget.h>
#include <ui_CalibrationWidget.h>


#include <sensor_msgs/Imu.h>

#include <imu_tk/base.h>



#include <QVector>
#include <QFile>
#include <QTextStream>

#include <vector>


enum TimestampUnit
{
  TIMESTAMP_UNIT_SEC = 1,
  TIMESTAMP_UNIT_MSEC = 1000,
  TIMESTAMP_UNIT_USEC = 1000000,
  TIMESTAMP_UNIT_NSEC = 1000000000
};

enum DatasetType
{
  DATASET_SPACE_SEPARATED,
  DATASET_COMMA_SEPARATED
};

class IMUCalibrationWidget : public BaseWidget, private Ui_CalibrationWidget
{
	Q_OBJECT
	Q_INTERFACES (WidgetPluginInterface)
	Q_PLUGIN_METADATA(IID "CalibrationWidget")

	public:

		template <typename _T>
		void readDataFromFile (  const QString& fileName, 
                      			std::vector< imu_tk::TriadData_<_T> > &samples0,
                      			std::vector< imu_tk::TriadData_<_T> > &samples1,
                      			TimestampUnit unit, DatasetType type );

		IMUCalibrationWidget (QWidget *parent = nullptr);
		virtual ~IMUCalibrationWidget ();

		virtual void init () override;

		virtual QWidget* createInstance () override;

	private:
		void imuCallback (const sensor_msgs::Imu::ConstPtr &imuMsg);
		void returnToMainWindow ();
		void setIcon (QString iconName);
		int createMessageBox (QString text, QString informativeText, bool button);
		bool supportedFileTypeCheck (QString path);
		void setAttitudeImage(QString imageName, QString nextImageName);
		
		double angular_velocity[3];
		double linear_acceleration[3];

		QFile dataFile;
		ros::Time startTime;

		bool readData = false;
		bool calibrate = false;
		bool started = false;

		QTimer timerOfProgressBar;
		QTimer timerOfInitializationPeriod;
		QTimer timerOfProgressText;
		QMessageBox msgBox;
		QString imuTopic;
		
		int counter = 0;
		int count = 1;
		double sumOfProgress = 0;
		double cycleTime = 0;
		double elapsedTime = 0;

	private Q_SLOTS:
		void on_startRecordButton_clicked ();
		void chooseFileButton_clicked (QLineEdit *path, QFileDialog::AcceptMode mode);
		void chooseDirectoryButton_clicked (QLineEdit *path, QFileDialog::AcceptMode mode);
		// void on_recordCancelButton_clicked ();
		void updateProgressBar ();
		void updateProgressText ();
		void on_startCalibrateButton_clicked ();
		// void on_calibrateCancelButton_clicked ();
		void startProgressTextTimer (); 
		void on_saveParameterButton_clicked ();
		void on_imuSelectionButton_clicked();
		// void on_resetImuButton_clicked();
		void on_resetCalibrationButton_clicked();
		
};

#endif /* SRC_IMU_CALIBRATER_GUI_CALIBRATIONWIDGET_H_ */
