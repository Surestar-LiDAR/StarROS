#include <cstdio>
#include <cstdint>
#include <vector>

namespace ss {
namespace calc {
namespace math {

class CTrackFile {
public:
    struct SBET_Rec { //3*5*8+2*8 =136
        double ti;			///< 时间，单位:周秒(time gps-sec. of week)
        double p[3];		///< 纬度 经度 高度 24
        double wVel[3];		///< X方向速度 Y方向速度 Z方向速度 24
        double rph[3];		///< roll, pitch, heading ,单位:弧度 24
        double w;			///< 游移方位角
        double force[3];		///< X方向加速度 Y方向加速度 Z方向加速度 24
        double aRate[3];		///< X方向角速度 Y方向角速度 Z方向角速度 24
    };

    typedef struct {
        SBET_Rec headInfo ;
        SBET_Rec tailInfo ;
        //double pageHeadTime ;
        //double pageTailTime ;
        uint32_t  pageStartOffset;
        uint32_t  pageSize ;
        int count ;
    }TRACK_PAGEINFO_S ;

    typedef struct {
        std::vector<TRACK_PAGEINFO_S> pageList ;
        double trackHeadTime ;
        double trackTailTime ;
        int currentIndex ;
    }TRACK_PAGE_LIST;

public:
    CTrackFile() ;
    ~CTrackFile() ;
    int setIndexFromTime(double mtTime) ;
    SBET_Rec getCurPosInfo() ;
    SBET_Rec getNextPosInfo() ;
    int LoadFile(const char* mtFileName) ;
    bool fileIsOpen() ;
    int CloseFile() ;

protected:
    int setCount(int mtCount ) ;
    int getCount() ;
    double PreTime() ;
    double CurTime() ;
    double nextTime() ;
    //double getHeadTime() ;
    //double getTailTime() ;
    int indexToNext() ;
    int setCurIndex(int mtIndex) ;
    int getIndexFromTime(double mtTime) ;
    double getTimeFromIndex(int mtIndex) ;
    SBET_Rec getInfoFromIndex( int mtIndex );
    void setDataBaseAddress(SBET_Rec *mtAdd) ;
    bool switchPage(int mtIndex);
    //test(int argc, char *argv[] );
private:
    SBET_Rec *m_infoGroup ;
    size_t m_count;
    int m_curIndex ;
    FILE *m_impfileHdl;
    bool m_fileIsOpen ;
    // void * m_imppvFile ;
    TRACK_PAGE_LIST m_trackInfo ;
};

}
}
}
