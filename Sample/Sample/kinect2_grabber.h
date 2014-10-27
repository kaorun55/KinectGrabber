// Kinect2Grabber is pcl::Grabber to retrieve the point cloud data from Kinect using Kinect for Windows SDK.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Visual Studio Professional以上を使う場合はCComPtrの利用を検討してください。
#include "ComPtr.h"
//#include <atlbase.h>

// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
        }


namespace pcl
{
    struct pcl::PointXYZ;
    struct pcl::PointXYZRGB;
    template <typename T> class pcl::PointCloud;

    class Kinect2Grabber : public pcl::Grabber
    {
    public:
        Kinect2Grabber( const int index = 0 );
        virtual ~Kinect2Grabber() throw ();
        virtual void start();
        virtual void stop();
        virtual bool isRunning() const;
        virtual std::string getName() const;
        virtual float getFramesPerSecond() const;

        typedef void (signal_Kinect_PointXYZ)( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>& );
        typedef void (signal_Kinect_PointXYZRGB)( const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>& );

    private:

        void updateColorFrame();
        void updateDepthFrame();

    protected:
        boost::signals2::signal<signal_Kinect_PointXYZ>* signal_PointXYZ;
        boost::signals2::signal<signal_Kinect_PointXYZRGB>* signal_PointXYZRGB;

        pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB();

        boost::thread thread;
        mutable boost::mutex mutex;

        void threadFunction();

        bool quit;
        bool running;

        HRESULT result;
        IKinectSensor* sensor = nullptr;
        ICoordinateMapper* mapper = nullptr;

        IColorFrameReader* colorFrameReader = nullptr;
        int colorWidth;
        int colorHeight;
        unsigned int colorBytesPerPixel;
        std::vector<BYTE> colorBuffer;

        IDepthFrameReader* depthFrameReader = nullptr;
        int depthWidth;
        int depthHeight;
        std::vector<UINT16> depthBuffer;
    };

    pcl::Kinect2Grabber::Kinect2Grabber( const int index )
        : sensor( nullptr )
        , mapper( nullptr )
        , result( S_OK )
        , running( false )
        , quit( false )
        , signal_PointXYZ( nullptr )
        , signal_PointXYZRGB( nullptr )
    {
        // Create Sensor Instance
        auto result = ::GetDefaultKinectSensor( &sensor );
        if ( FAILED( result ) ){
            throw std::exception( "Exception : GetDefaultKinectSensor" );
        }

        // Open Sensor
        result = sensor->Open();
        if ( FAILED( result ) ){
            throw std::exception( "Exception : IKinectSensor::Open" );
        }

        // Retrieved Coordinate Mapper
        result = sensor->get_CoordinateMapper( &mapper );
        if ( FAILED( result ) ){
            throw std::exception( "Exception : IKinectSensor::get_CoordinateMapper" );
        }

        // Retrieved Image Size from Stream Resolution
        ComPtr<IColorFrameSource> colorFrameSource;
        ERROR_CHECK( sensor->get_ColorFrameSource( &colorFrameSource ) );

        ComPtr<IFrameDescription> colorFrameDescription;
        ERROR_CHECK( colorFrameSource->CreateFrameDescription( ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
        ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
        ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
        ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) );

        colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );

        // Depth画像のサイズを取得する
        ComPtr<IDepthFrameSource> depthFrameSource;
        ERROR_CHECK( sensor->get_DepthFrameSource( &depthFrameSource ) );

        ComPtr<IFrameDescription> depthFrameDescription;
        ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
        ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
        ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );

        depthBuffer.resize( depthWidth * depthHeight );

        signal_PointXYZ = createSignal<signal_Kinect_PointXYZ>();
        signal_PointXYZRGB = createSignal<signal_Kinect_PointXYZRGB>();
    }

    pcl::Kinect2Grabber::~Kinect2Grabber() throw()
    {
        stop();

        disconnect_all_slots<signal_Kinect_PointXYZ>();
        disconnect_all_slots<signal_Kinect_PointXYZRGB>();

        // End Processing
        sensor->Close();
        mapper->Release();

        thread.join();
    }

    void pcl::Kinect2Grabber::start()
    {
        // カラーリーダーを取得する
        ComPtr<IColorFrameSource> colorFrameSource;
        ERROR_CHECK( sensor->get_ColorFrameSource( &colorFrameSource ) );
        ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

        // Depthリーダーを取得する
        ComPtr<IDepthFrameSource> depthFrameSource;
        ERROR_CHECK( sensor->get_DepthFrameSource( &depthFrameSource ) );
        ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

        running = true;

        thread = boost::thread( &Kinect2Grabber::threadFunction, this );
    }

    void pcl::Kinect2Grabber::stop()
    {
        boost::unique_lock<boost::mutex> lock( mutex );

        quit = true;
        running = false;

        lock.unlock();
    }

    bool pcl::Kinect2Grabber::isRunning() const
    {
        boost::unique_lock<boost::mutex> lock( mutex );

        return running;

        lock.unlock();
    }

    std::string pcl::Kinect2Grabber::getName() const{
        return std::string( "Kinect2Grabber" );
    }

    float pcl::Kinect2Grabber::getFramesPerSecond() const {
        return 30.0f;
    }

    // カラーフレームの更新
    void pcl::Kinect2Grabber::updateColorFrame()
    {
        // フレームを取得する
        ComPtr<IColorFrame> colorFrame;
        auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
        if ( ret != S_OK ){
            return;
        }

        // BGRAの形式でデータを取得する
        ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
            colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
    }

    void pcl::Kinect2Grabber::updateDepthFrame()
    {
        // Depthフレームを取得する
        ComPtr<IDepthFrame> depthFrame;
        auto ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
        if ( ret != S_OK ){
            return;
        }

        // データを取得する
        ERROR_CHECK( depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] ) );
    }

    void pcl::Kinect2Grabber::threadFunction()
    {
        while ( !quit ){
            boost::unique_lock<boost::mutex> lock( mutex );

            updateColorFrame();
            updateDepthFrame();

            lock.unlock();

            if ( signal_PointXYZ->num_slots() > 0 ) {
                signal_PointXYZ->operator()( convertDepthToPointXYZ() );
            }

            if ( signal_PointXYZRGB->num_slots() > 0 ) {
                signal_PointXYZRGB->operator()( convertRGBDepthToPointXYZRGB() );
            }
        }
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::Kinect2Grabber::convertDepthToPointXYZ()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );

        cloud->width = static_cast<uint32_t>(depthWidth);
        cloud->height = static_cast<uint32_t>(depthHeight);
        cloud->is_dense = false;

        for ( int y = 0; y < depthHeight; y++ ){
            for ( int x = 0; x < depthWidth; x++ ){
                pcl::PointXYZ point;

                DepthSpacePoint depthPoint;
                depthPoint.X = x;
                depthPoint.Y = y;

                // Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
                CameraSpacePoint cameraPoint;
                mapper->MapDepthPointToCameraSpace( depthPoint, depthBuffer[y * depthWidth + x], &cameraPoint );

                point.x = cameraPoint.X;
                point.y = cameraPoint.Y;
                point.z = cameraPoint.Z;

                cloud->push_back( point );
            }
        }

        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::Kinect2Grabber::convertRGBDepthToPointXYZRGB()
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );

        //cloud->width = static_cast<uint32_t>(width);
        //cloud->height = static_cast<uint32_t>(height);
        //cloud->is_dense = false;

        //NUI_DEPTH_IMAGE_PIXEL* depthPixel = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(depthLockedRect->pBits);

        //for ( int y = 0; y < height; y++ ){
        //    for ( int x = 0; x < width; x++ ){
        //        pcl::PointXYZRGB point;

        //        NUI_DEPTH_IMAGE_POINT depthPoint;
        //        depthPoint.x = x;
        //        depthPoint.y = y;
        //        depthPoint.depth = depthPixel[y * width + x].depth;

        //        // Coordinate Mapping Depth to Real Space, and Setting PointCloud XYZ
        //        Vector4 skeletonPoint;
        //        mapper->MapDepthPointToSkeletonPoint( NUI_IMAGE_RESOLUTION_640x480, &depthPoint, &skeletonPoint );

        //        point.x = skeletonPoint.x;
        //        point.y = skeletonPoint.y;
        //        point.z = skeletonPoint.z;

        //        // Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
        //        NUI_COLOR_IMAGE_POINT colorPoint;
        //        mapper->MapDepthPointToColorPoint( NUI_IMAGE_RESOLUTION_640x480, &depthPoint, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, &colorPoint );

        //        if ( 0 <= colorPoint.x && colorPoint.x < width && 0 <= colorPoint.y && colorPoint.y < height ){
        //            unsigned int index = colorPoint.y * colorLockedRect->Pitch + colorPoint.x * 4;
        //            point.b = colorLockedRect->pBits[index + 0];
        //            point.g = colorLockedRect->pBits[index + 1];
        //            point.r = colorLockedRect->pBits[index + 2];
        //        }

        //        cloud->push_back( point );
        //    }
        //}

        return cloud;
    }
}

#endif KINECT2_GRABBER

