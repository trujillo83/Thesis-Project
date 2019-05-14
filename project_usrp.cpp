#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <sys/types.h> // open, fstat
#include <sys/stat.h> // open, fstat
#include <sys/mman.h>  // mmap, munmap
#include <iostream>
#include <fstream>
#include <complex>
#include <csignal>
#include <chrono>
#include <thread>
#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <pthread.h>
#include <future>
#include <assert.h>
#include <ctime>
#include <cstdlib>
#include <iomanip> // setprecision
#include <vector>
#include <dirent.h>
#include <complex>
#include<fftw3.h>
#define BAUDRATE 115200;
#define REAL 0
#define IMAG 1
namespace po = boost::program_options;
using namespace std;
mutex mtx;           // mutex for critical section
using chrono::system_clock;
typedef chrono::system_clock Clock;
//global variables:
    static bool stop_signal_called = false ;
    bool repeat = false, saveCSV=false;
    int Nports=100;//check 100 ports
    string IDS[100],dir;
    signed long int total_samples[100];
    long double Averages[100];
    double duration_thread[100],difference_file_thread[100],duration_file,total_duration=0.0;
    auto start_file = std::chrono::steady_clock::now();
    auto end_file = std::chrono::steady_clock::now();
    int baudrate = BAUDRATE;
    //int BLAS=0;
    char DATE[20];
//FUNCTION DECLARATIONS
    void sig_int_handler(int){stop_signal_called = true;}
    //void repeat_N_times() {repeat= false;}
    int  serialport_init(const char* serialport, int baud);
    void checkid(string ID,long double sum_thread);
    void checkTotalHarvested(void);
    void checktime_file(double check_time);
    void checktime_thread(void);
    int  checkfot(string ID);
    void writeSummary(string fileName, int N_check);
    void check_diff(void);
    void check_numberSamples(int k, string IDPH);
    //vector<complex<double>> makeVector( const std::string &file,int length);
    vector<string> check_ports(int NN,int baudrate);
    void progress (int j,bool inter);
    void map_file(char *fname, size_t *flen);
    void transmissionfunction(uhd::tx_streamer::sptr tx_stream, vector<complex<double>> BUFF, size_t spb);
    string set_time(double duration);
    int getdir (string dir, vector<string> &files);
    void CheckDot(vector<string> &files);
    void countdown();
    bool is_number(const std::string& s);
    void creatingFolders();
    vector<complex<double>> randomVector(int totalN);
    void ifft(fftw_complex *in, fftw_complex *out,int NN);
    void fft(fftw_complex *in, fftw_complex *out,int NN);
    void assignfft( vector<complex<double>> randomVector, int NN, vector<complex<double>> *vectorIFFT  );
    void finalMessage();
    void normalized(vector<complex<double>> inVector,vector<complex<double>> *Yvector);
    void displayComplex(fftw_complex *y,fftw_complex *x,vector<complex<double>> randomVector,vector<complex<double>> IFFT);
    void PowerVector(vector<complex<double>> inVector,double *Power,double *PAPR);
//STRUCTURE//
    struct Stuff{string name;string value;string default_value;};
    struct BUFFER{string IDPH;string ADCport;string data;};    
//*****************************************************************************THREADS***********************************************************************************************//
void read_serial (string ard,string file,future<void> futureObj){
    time_t tt;
    long double sum,prev;
    char serialport[256],b[1];
    char buf[1024];
    int fd,i=0,j=0,k=0;
    long int l=0;
    BUFFER buffer;
    strcpy (serialport,ard.c_str());
    fd=serialport_init(serialport,baudrate);
    if(fd==-1){cout<<endl<<"\033[1;31m[Not able to start thread for port :\033[0m  \033[7;37m"<<ard<<"\033[0m \033[1;31m, not connected or does not exist]\033[0m\n"<<endl; return;}
    int wasteLines=10;     //skip first 2 lines, discard error
    ofstream myfile;
    string DATE_s(DATE);
    do {
    int n = read(fd, b, 1);  // read a char at a time
    if( n==-1) continue;    // couldn't read
    if( n==0 ) continue;
    buf[i] = b[0]; i++;
    if (b[0] =='\n') {buf[i] = 0;i=0; 
    if(j==wasteLines-1){ string buff=buf;buffer.IDPH=buff;buffer.IDPH.erase(9,31);}
    j++;
    }
    }while(j<wasteLines);
    string IDPH=buffer.IDPH;
    if(saveCSV){  
    file.erase(file.end()-4,file.end());
    string file_name= "../CSV_files/"+DATE_s+"/"+IDPH+"_"+file+".csv";    
    myfile.open (file_name);
    myfile << "OUTPUTS from ID: "<<IDPH<<","<<file<<"\n";
    myfile<<" Time , Output(mW) \n";
    }
    auto start_thread = std::chrono::steady_clock::now();
    do { 
    int n = read(fd, b, 1);  // read a char at a time
    if( n==-1) continue;    // couldn't read
    if( n==0 ) continue;	
    buf[i] = b[0]; i++;
     if (b[0] =='\n') {		
    buf[i] = 0;  // null terminate the string
    i=0;
    if(k<=1){ 
    k++; continue;}
    string buff=buf; 
    //cout<<buff;  
    size_t found1 = buff.find(":"), found2 = buff.find("."),found3 = buff.find(".",found2+1), found4 = buff.find(":",found1+1);    
    if(found1!=-1 && found2!=-1 && found3==-1 && found4==-1) {
    buffer.data=buff;
    buffer.data.erase(0,12);
    buffer.data.erase(buffer.data.end()-1,buffer.data.end());
    try{
    prev=stold(buffer.data);
    }
    catch (exception e) {continue;} 
    l++;
    }
    else continue;
    sum+= prev;    
    if(saveCSV){
    system_clock::time_point today = system_clock::now();
    tt = system_clock::to_time_t ( today );
    auto now = Clock::now();
    auto seconds = chrono::time_point_cast<chrono::seconds>(now);
    auto fraction = now - seconds; 
    tm local_tm= *localtime(&tt);
    auto milliseconds = chrono::duration_cast<std::chrono::milliseconds>(fraction);  
    //  cout<<buffer.IDPH<<" "<<buffer.data<<endl;
     myfile <<local_tm.tm_hour<<":"<<local_tm.tm_min<<":"<<local_tm.tm_sec<<"."<<milliseconds.count()<<","<<buffer.data<<"\n";}
    }
    } while(futureObj.wait_for(chrono::	(300)) == future_status::timeout);
    string port_num_s;
    port_num_s=ard.erase(0,11); 
    int port_num_i=stoi(port_num_s);
    IDS[port_num_i]=IDPH; 
    auto average=sum/l;
    check_numberSamples(l,IDPH);
    checkid(IDPH,average);
    auto end_thread = chrono::steady_clock::now();
    chrono::duration<double> diff_thread = end_thread-start_thread;
    double duration_t=diff_thread.count();
    int fot = checkfot(IDPH);
    duration_thread[fot]=duration_t;
    }

void read_serialVectorMode (string ard,future<void> futureObj){
    time_t tt;
    long double sum,prev;
    char serialport[256],b[1];
    char buf[1024];
    int fd,i=0,j=0,k=0;
    long int l=0;
    BUFFER buffer;
    strcpy (serialport,ard.c_str());
    fd=serialport_init(serialport,baudrate);
    if(fd==-1){cout<<endl<<"\033[1;31m[Not able to start thread for port :\033[0m  \033[7;37m"<<ard<<"\033[0m \033[1;31m, not connected or does not exist]\033[0m\n"<<endl; return;}
    //skip first 3 lines, discard error
    int wasteLines=3;
    do {
    int n = read(fd, b, 1);  // read a char at a time
    if( n==-1) continue;    // couldn't read
    if( n==0 ) continue;
    buf[i] = b[0]; i++;
    if (b[0] =='\n') {buf[i] = 0;i=0; 
    if(j==wasteLines-1){ string buff=buf;buffer.IDPH=buff;buffer.IDPH.erase(9,31);}
    j++;
    }
    }while(j<wasteLines);
    string IDPH=buffer.IDPH;
    do { 
    int n = read(fd, b, 1);  // read a char at a time
    if( n==-1) continue;    // couldn't read
    if( n==0 ) continue;	
    buf[i] = b[0]; i++;
    if (b[0] =='\n') {		
    buf[i] = 0;  // null terminate the string
    i=0;
    if(k<=1){k++; continue;}
    string buff=buf; 
    //cout<<buff;  
    size_t found1 = buff.find(":"), found2 = buff.find("."),found3 = buff.find(".",found2+1), found4 = buff.find(":",found1+1);    
    if(found1!=-1 && found2!=-1 && found3==-1 && found4==-1) {
    buffer.data=buff;
    buffer.data.erase(0,12);
    buffer.data.erase(buffer.data.end()-1,buffer.data.end());
    try{ prev=stold(buffer.data);}
    catch (exception e) {continue;} 
    l++;
    }
    else continue;
    sum+= prev;}
    } while(futureObj.wait_for(chrono::microseconds(1)) == future_status::timeout);
    string port_num_s;
    port_num_s=ard.erase(0,11); 
    int port_num_i=stoi(port_num_s);
    IDS[port_num_i]=IDPH; 
    auto average=sum/l;
    check_numberSamples(l,IDPH);
    checkid(IDPH,average);
    int fot = checkfot(IDPH);
    }
//**************************************TEMPLATES***********************************************************************************************//

template<typename samp_type> void transmission(uhd::tx_streamer::sptr tx_stream, vector<samp_type> BUFF, size_t spb){
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false; 
   // bool inter=true;
    //int j=0,Porcentage=20;/
    int counter=0,totalSPB=0;
    samp_type *p;
    samp_type* end = &BUFF.back(); 
    //samp_type* start= &BUFF.front();
    for(int i =0;i<BUFF.size();i+=spb){
    signal(SIGINT, &sig_int_handler);
    //string progress_L =  string(j, '*') + string(Porcentage - j, ' ') ; string progress_R =  string(Porcentage - j, ' ')+ string(j, '*')   ;
    //if (inter==true){ cout <<string(20,' ')<<  "\r \t \033[1;33m[" << progress_L <<    "SENDING SIGNAL"<< progress_R<<"] \033[0m"<<string(20,' ') <<flush; inter=false;}
    //else if (inter==false){cout<<string(20,' ') <<  "\r\t \033[1;33m[" << progress_L <<"SENDING SIGNAL"<< progress_R<<"] \033[0m"<<string(20,' ') << flush;inter=true;}	
    p=&BUFF.at(i);
    if(p+spb>end) {
    spb =(end-p)+1;
    //totalSPB+=spb*sizeof(samp_type);
    md.end_of_burst=true;
    //tx_stream->send(&BUFF.at(i), spb, md);
  //return;
} 
  //cout<<counter<<")"<<" SAMPLES PER BUFFER = "<<spb<<" Samples sent = "<<spb*sizeof(complex<float>)<<endl;
    
//cout<<"address for p : "<<p<<endl<<"address for end : "<<end<<"LAST SPB = "<<spb<<endl<<"end-p = "<<end-p<<endl<<"TOTAL SAMPLES SENT :"<<totalSPB<<endl;
   tx_stream->send(&BUFF.at(i), spb, md);
  // cout<<"num_tx_samp = "<<spb<<endl;
   totalSPB+=spb*sizeof(complex<double>);    
   counter+=1;
    //j+=2;
    if (stop_signal_called==true) {//cout<<"\r\033[F"<<flush; 
    return;}
    if(md.end_of_burst==true){//cout<<"\r\033[F"<<flush;
  //  cout<<"Total Number of samples sent = "<<totalSPB<<endl;
   //cout<<" COUNTER = "<<counter<<endl;


return;}
    //if (j==20)j=0;
    }
}

template<typename samp_type> vector<samp_type> makeVector( const string &file,int length){
    bool endOfFile=false;
    int size_of_vector= length/sizeof(samp_type);
    cout<<"\tsize_of_vector = "<<size_of_vector<<string(120,' ')<<endl;
    
vector<samp_type> buff(size_of_vector);    

    cout<<"\tsize_of_buff = "<< buff.size()<<endl;
    cout<<"\tlength = "<<length<<endl;
    ifstream infile(file.c_str(), std::ifstream::binary);
    infile.read((char*)&buff.front(), length);
    infile.close();
    //for(int i=0;i<50;i++){cout<<"["<<i<<"] : "<<buff[i]<<endl;}
    return buff;


}


template<typename samp_type> void normalized(vector<samp_type> inVector,vector<samp_type> *Yvector,double *Powerin,double *Powerout,double *PAPR){
    size_t Ntotal= inVector.size();
    double mod,sum=0,max=0;  
    for(int i=0;i<Ntotal;i++){
    //cout<<inVector[i]<<endl;
    mod=pow(real(inVector[i]),2)+pow(imag(inVector[i]),2);
    if(mod>max)max=mod;
    sum+=mod;
    }
    *PAPR=max/sum;
    *Powerin=sum/Ntotal;
    
  //  PowerVector<samp_type>(inVector,&P,&PAPR);
    for(int i=0;i<Ntotal;i++){
    inVector[i]/=2*sqrt(*Powerin); // DIVIDING TWICE THE POWER
    }
    *Yvector=inVector;
     sum=0;
     mod=0;
     for(int i=0;i<Ntotal;i++){
    //cout<<inVector[i]<<endl;
    mod=pow(real(inVector[i]),2)+pow(imag(inVector[i]),2);
    sum+=mod;
    }
    *Powerout=sum/Ntotal;
}

template<typename samp_type> void send_from_file(
    uhd::tx_streamer::sptr tx_stream,
    const std::string &file,
    size_t samps_per_buff
){
    int AUX=0;
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;
    std::vector<samp_type> buff(samps_per_buff);
    std::ifstream infile(file.c_str(), std::ifstream::binary);

    //loop until the entire file has been read

    while(not md.end_of_burst and not stop_signal_called){

        infile.read((char*)&buff.front(), buff.size()*sizeof(samp_type));
        size_t num_tx_samps = size_t(infile.gcount()/sizeof(samp_type));
        md.end_of_burst = infile.eof();
        

        tx_stream->send(&buff.front(), num_tx_samps, md);
       //std::cout<<"num_tx_samps = "<<num_tx_samps<<std::endl;

       /*
        if(md.end_of_burst==true){
       for(int i=0;i<buff.size();i++){
        std::cout<<"["<<i<<"] : "<<buff[i]<<std::endl;}
        }*/
    }

    infile.close();
}




template<typename samp_type> void getPower( vector<samp_type> inVector, double *Power,double *PAPR){
    size_t Ntotal= inVector.size();
    double mod,sum=0,max=0;
    for(int i=0;i<Ntotal;i++){
    mod=pow(real(inVector[i]),2)+pow(imag(inVector[i]),2);
    if(mod>max)max=mod;
    sum+=mod;
    }
    *PAPR=max/sum;
    *Power=sum/Ntotal;

}
//*************************************************************************MAIN*****************************************************************************//
int UHD_SAFE_MAIN(int argc, char *argv[]){ 
    Stuff stuff[256];
    fstream fin("../Parameters_USRP.txt");
    int nb_entries,j,N_times;
    string title;
    getline(fin,title,'\t');
    for (j=0 ;j<6;j++){string wasted_lines;	getline(fin,wasted_lines);}
    for (nb_entries=0;fin.good() && nb_entries < 256 ; nb_entries++){
    fin>>stuff[nb_entries].name;
    fin>>stuff[nb_entries].value;
    fin>>stuff[nb_entries].default_value;
    }
    Stuff * pts[nb_entries];
    for(int i=0;i<nb_entries-1;i++){
    pts[i]=&stuff[i];
    }  
    if(pts[9]->value.compare("ON")==0 || pts[9]->default_value.compare("ON")==0) saveCSV=true;
    if(pts[11]->value.compare("ON")!=0|| pts[11]->default_value.compare("OFF")==0) creatingFolders();

    //variables to be set by po
    std::string args, file, type, ant, subdev, ref, wirefmt, channel,ard;
    int spb;
    double rate, freq, bw,gain, delay, lo_off;
    //setup the program options
    po::options_description desc("Allowed options, check Parameters_USRP.txt file");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
       	("dir", po::value<std::string>(&dir)->default_value(pts[5]->value), "name of the directory to read binary samples files from")
        ("type", po::value<std::string>(&type), "sample type: double, float, or short")
        ("spb", po::value<int>(&spb), "samples per buffer")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("lo_off", po::value<double>(&lo_off), "Offset for frontend LO in Hz (optional)")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8 or sc16)")
        ("delay", po::value<double>(&delay)->default_value(0.0), "specify a delay between repeated transmission of file (in seconds)")
        ("channel", po::value<std::string>(&channel)->default_value("0"), "which channel to use")
        ("repeat", "repeatedly transmit file")
        ("int-n", "tune USRP with integer-n tuning") ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    //print the help message
    if (vm.count("help")){
    std::cout << boost::format("UHD TX samples from file %s") % desc << std::endl;
    return ~0;}
    //create a usrp device
    std::cout << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    //Lock mboard clocks
    usrp->set_clock_source(ref);
    cout<<endl<<"\033[1;32m[SETTING PARAMETERS FROM FILE: "<<title<<"] \033[0m"<<endl<<endl;    
    //cout<<" MODE VECTOR TRANSMISSION ON "<<endl;
    if (vm.count("subdev")) usrp->set_tx_subdev_spec(subdev);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;
    //set the sample rate
    if (not vm.count("rate")){
    if (pts[2]->value=="-"&& pts[2]->default_value=="-"){std::cerr << "Please specify the sample rate with --rate or edit "<<title << std::endl;return ~0;}
    if (pts[2]->value=="-") rate=stod(pts[2]->default_value);
    else rate=stod(pts[2]->value);}
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;
    //set the center frequency
    if (not vm.count("freq")){
    if (pts[3]->value=="-"&& pts[3]->default_value=="-"){std::cerr << "Please specify the center frequency with --freq or edit "<<title << std::endl;return ~0;}
    if (pts[3]->value=="-") freq=stod(pts[3]->default_value);
    else freq=stod(pts[3]->value);}
    std::cout << boost::format("Setting TX Freq: %f MHz...") % (freq/1e6) << std::endl;
    uhd::tune_request_t tune_request;
    if(vm.count("lo_off")) tune_request = uhd::tune_request_t(freq, lo_off);
    else tune_request = uhd::tune_request_t(freq);
    if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
    usrp->set_tx_freq(tune_request);
    std::cout << boost::format("Actual TX Freq: %f MHz...") % (usrp->get_tx_freq()/1e6) << std::endl << std::endl;
    //set the rf gain
    if (not vm.count("gain")){
    if (pts[4]->value=="-"&& pts[4]->default_value=="-"){
    cerr << "Please specify the gain with --gain or edit "<<title<<endl;
    return ~0;}
    if (pts[4]->value=="-") gain=stod(pts[4]->default_value);
    else gain=stod(pts[4]->value);}
    std::cout << boost::format("Setting TX Gain: %f dB...") % gain << std::endl;
    usrp->set_tx_gain(gain);
    std::cout << boost::format("Actual TX Gain: %f dB...") % usrp->get_tx_gain() << std::endl << std::endl;

    //set the analog frontend filter bandwidth
    if (not vm.count("bw")){
   
    if (pts[6]->value=="-"&& pts[6]->default_value=="-"){
    cerr << "Please specify the Bandwidth with --bw or edit "<<title<<endl;
    return ~0;}
    if (pts[6]->value=="-") bw=stod(pts[6]->default_value);
    else bw=stod(pts[6]->value);}
    std::cout << boost::format("Setting TX Bandwidth: %f MHz...")% (bw / 1e6)<< std::endl;
    usrp->set_tx_bandwidth(bw);
    std::cout << boost::format("Actual TX Bandwidth: %f MHz...")% (usrp->get_tx_bandwidth() / 1e6)<< std::endl << std::endl;
    


    //set the antenna
    if (vm.count("ant")) usrp->set_tx_antenna(ant);
    //allow for some setup time:
    std::this_thread::sleep_for(std::chrono::seconds(1));
    //Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
    uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked",0);
    std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
    UHD_ASSERT_THROW(lo_locked.to_bool());}
    sensor_names = usrp->get_mboard_sensor_names(0);
    if ((ref == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
    uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked",0);
    std::cout << boost::format("Checking TX: %s ...") % mimo_locked.to_pp_string() << std::endl;
    UHD_ASSERT_THROW(mimo_locked.to_bool());}
    if ((ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
    uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked",0);
    std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string() << std::endl;
    UHD_ASSERT_THROW(ref_locked.to_bool());}
    //set sigint if user wants to receive
    if(pts[7]->value.compare("-")!=0){repeat= true; N_times = stoi(pts[7]->value);}
    else{ N_times=stoi(pts[7]->default_value);}
    //type
    if(not vm.count("type")){
    if (pts[0]->value=="-"&& pts[0]->default_value=="-"){ 
    cerr << "Please specify the sample type with --type or edit "<<title<<endl;return ~0;}
    if (pts[0]->value=="-") type=pts[0]->default_value;
    else type=pts[0]->value;}
    //LIST DIRECTORIES
    vector<string> files = vector<string>();
    getdir(dir,files);
    if(files.size()==0){cout<<endl<<"\033[1;31mDIRECTORY\033[0m "<<dir<<"\033[1;31m DOES NOT CONTAIN ANY FILE TO TRANSMIT!! 	mak \033[0m"<<endl;return EXIT_FAILURE;}
    if(pts[11]->value.compare("ON")!=0){
    cout<<endl<<"\033[1;32m FILES IN DIRECTORY :\033[0m"<<endl;
    for (unsigned int i = 0;i < files.size();i++) {
    cout<<"\033[1;34m\t -------------------->\033[0m "<<files[i]<<endl;
    //this_thread::sleep_for (chrono::milliseconds(500)); 
    }   
    }  
  if(pts[11]->value.compare("ON") ==0) {
    cout<<"\033[1;32m"<<string(30,'>')<<"\033[0m"<<"\033[1;34m[MODE<VECTOR_TRANSMISSION>  ON ]\033[0m"<<"\033[1;32m"<<string(30,'<')<<"\033[0m"<<endl; 
    if(pts[13]->value.compare("ON")==0 && pts[12]->value.compare("ON")!=0 ){
    cout<<"\033[1;32m"<<string(30,'>')<<"\033[0m"<<"\033[1;34m[SUBMODE<ITERATION>         ON ]\033[0m"<<"\033[1;32m"<<string(30,'<')<<"\033[0m"<<endl<<endl;}
    else  cout<<"\033[1;32m"<<string(30,'>')<<"\033[0m"<<"\033[1;34m[REPEAT_TX                 ON ]\033[0m"<<"\033[1;32m"<<string(30,'<')<<"\033[0m"<<endl<<endl;
    }

    //create a transmit streamer
    std::string cpu_format;
    std::vector<size_t> channel_nums;
    if (type == "double") cpu_format = "fc64";
    else if (type == "float") cpu_format = "fc32";
    else if (type == "short") cpu_format = "sc16";
    uhd::stream_args_t stream_args(cpu_format, wirefmt);
    channel_nums.push_back(boost::lexical_cast<size_t>(channel));
    stream_args.channels = channel_nums;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);
    //Samples_per_buffer (spb)
    if(not vm.count("spb")){
    if (pts[1]->value=="-"&& pts[1]->default_value=="-"){    
    cerr << "Please specify the samples per buffer with --spb  or edit "<<title<<endl;
    return ~0;}
    if (pts[1]->value=="-") spb=stoi(pts[1]->default_value);
    else spb=stoi(pts[1]->value);}    
    //Arduino
    vector<string> ports= check_ports(Nports,baudrate);//check ports
    int sizeVec=ports.size();
    cout<<endl<<"\033[1;32m ARDUINO PORTS CONNECTED : \033[0m"<<endl;
    if(sizeVec==0) cout<<"\t\033[1;31mNO ARDUINO CONNECTED!!\033[0m"<<endl;
    for (int i=0;i<sizeVec;i++){
    cout<<"\033[1;34m\t -------------------->\033[0m "<<ports[i]<<endl;
      //this_thread::sleep_for (chrono::milliseconds(500)); 
    }
    cout<<endl<<"\033[1;32m SAVING SAMPLING TRACK IN CSV FILES\033[0m"<<endl<<"\033[1;34m\t -------------------->\033[0m ";
    if(saveCSV && sizeVec!=0) cout<<"ON"<<endl;
    else cout<<"OFF"<<endl;
    bool AUX=false;
   
    if(pts[11]->value.compare("ON")==0) goto VECTOR_TX;
    int N_FILES;
    if(pts[8]->value=="-"){ N_FILES=files.size();}
    else{ N_FILES=1;
    AUX=true;
    cout<<endl<<"\033[1;35mTRANSMIT FILE "<<pts[8]->value<<" SELECTED \033[0m"<<endl<<endl;}
    //countdown();
    for (int i=0;i<N_FILES;i++){
    string nameOfFile= files[i];
    string file= dir+"/"+files[i];
    if (AUX==true){file= dir+"/"+pts[8]->value;}    
    else{nameOfFile= files[i];
    file= dir+"/"+files[i];}
    size_t len;
    char *fname=strdup(file.c_str());
    double Powerin,Powerout,PAPR;
    map_file(fname, &len);
    
//cout<<"FILE : "<<file<<string(120,' ')<<endl;
vector<complex<double>> buffD;
vector<complex<float>> buffF;
vector<complex<short>> buffS;

     if (type == "double")         buffD = makeVector<complex<double>>(file,len);
        else if (type == "float")  buffF= makeVector<complex<float>>(file,len);
        else if (type == "short")  buffS= makeVector<complex<short>>(file,len);
        else throw std::runtime_error("Unknown type " + type);

/*
for(int i=0;i<buffD.size();i++){

cout<<i<<"] "<<buffD[i]<<endl;
}


return 0 	;
*/
/*
cout<<"BUFFER"<<endl;
for(int i=0;i<10;i++){

cout<<i<<"] "<<buffD[i]<<endl;
}

*/
vector<complex<double>> normalizedVectorD;
vector<complex<float>> normalizedVectorF;
vector<complex<short>> normalizedVectorS;

     if (type == "double")         normalized <complex<double>> (buffD, &normalizedVectorD,&Powerin,&Powerout,&PAPR);
        else if (type == "float")  normalized <complex<float>> (buffF, &normalizedVectorF,&Powerin,&Powerout,&PAPR);
        else if (type == "short")  normalized <complex<short>> (buffS, &normalizedVectorS,&Powerin,&Powerout,&PAPR);
        else throw std::runtime_error("Unknown type " + type);


    //normalized(buff, &normalizedVector);	
    //PowerVector(normalizedVector,&P,&PAPR);
    cout<<"\tPOWER vectorIFFT: "<< Powerin<< " PAPR : " <<PAPR<<endl;
    cout<<"\tPOWER normalized: "<< Powerout<< " PAPR : " <<PAPR<<endl;
  

   int powerdbm=stoi(pts[16]->value);
   double powerlinear=pow(10,(0.1*(powerdbm-30)));
   cout<<"POWER = "<<powerdbm<<" dBm"<<endl;
   cout<<"POWER = "<<powerlinear<<" watts"<<endl;
/*
if(pts[17]->value.compare("ON")==0){
   for(int i=0;i<normalizedVectorD.size();i++){
   normalizedVectorD[i]*=sqrt(powerlinear);
   }
  //buffD=normalizedVectorD;
}*/
  // double powerNorm,PAPRnorm;
  // getPower<complex<double>>(normalizedVectorD,&powerNorm,&PAPRnorm);
//   cout<<"POWER of norm vector times sqrt("<<powerlinear<<" watts) = "<<powerNorm<<" watts, and PAPR = "<<PAPRnorm<<endl;

/*
cout<<"NORM"<<endl;	 
for(int i=0;i<10;i++){

cout<<i<<"] "<<normalizedVectorD[i]<<endl;
}
*/
//return 0;
    promise<void> exitSignal[sizeVec]; // future and promise for threads
    future<void> futureObj[sizeVec];
    thread threads[sizeVec];
    for (int i = 0; i < sizeVec; i++) { // spawn n threads:
    futureObj[i]=exitSignal[i].get_future();
    threads[i] = thread(&read_serial,ports[i],nameOfFile,move(futureObj[i]));}
    start_file = std::chrono::steady_clock::now();
    //Start transmission
    int LT= 0;
    if(N_times==0)
    {cout<<"TRANSMISSION TIMES = 0, CHECK PARAMETERS!!"<<endl;return EXIT_SUCCESS;}
  
/*
//send from file
    do{
        if (type == "double") send_from_file<std::complex<double> >(tx_stream, file, spb);
        else if (type == "float") send_from_file<std::complex<float> >(tx_stream, file, spb);
        else if (type == "short") send_from_file<std::complex<short> >(tx_stream, file, spb);
        else throw std::runtime_error("Unknown type " + type);

        if(repeat and delay > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(int64_t(delay*1000))
            );
        }
    } while(repeat and not stop_signal_called);
*/
	if(N_times>1){
         std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
	}

    do{
    //cout<<"\r \033[1;33mTransmission Number***"<<LT+1<<"***of***"<<N_times<<"**\t\033[0m"<<"for file : "<<file<<flush<<endl;
   // if (type == "double") 
   transmissionfunction(tx_stream, buffD, spb);
   //cout<<"\r\033[F"<<flush;
     //   else if (type == "float") transmission <complex<float> >(tx_stream, buffF, spb);
       // else if (type == "short") transmission <complex<short> >(tx_stream, buffS, spb);
       // else throw std::runtime_error("Unknown type " + type);
    //transmission(tx_stream, buff, spb);
    //if(repeat and delay > 0.0) {
    //this_thread::sleep_for(
    //std::chrono::milliseconds(int64_t(delay*1000)));}
    LT+=1;
    } while(LT!=N_times and not stop_signal_called);


    end_file = chrono::steady_clock::now();
    chrono::duration<double> diff_file = end_file-start_file;
    

    //end n threads
    for(int i=0;i<sizeVec;i++){
    exitSignal[i].set_value();
    threads[i].join(); }
    duration_file=diff_file.count();
    total_duration+=duration_file;
    check_diff();
    writeSummary(nameOfFile,LT);} 

    cout<<endl;
   
    finalMessage();
       checkTotalHarvested();
    //cout<<BLAS<<" times when stold read a non number string!!"<<endl;
return EXIT_SUCCESS;
   


////////////////////////////////
VECTOR_TX:
    
   double Powerin,Powerout,PAPR;
   int TL=pow(2,stoi(pts[15]->value)),Nwindow,counter=0;

   size_t spbVect=stoi(pts[1]->value);
   if(pts[12]->value.compare("ON")!=0)repeat=false;
   try{Nwindow=stoi(pts[14]->value);}
   catch(exception e){
   cout<<"Iterative_Window(>100) must be an integer number"<<endl;
   cout<<"Iterative_Window(>100) set to :"<<pts[14]->default_value<<endl;
   Nwindow=stoi(pts[14]->default_value);}
   
   vector<complex<double>> someVector= randomVector(TL);


/*
int KK=someVector.size();
 for(int i=0;i<someVector.size();i++){
  
  if (i<1 || i>200){ someVector[i]=0;KK--;}
// cout<<i<<") "<<someVector[i]<<endl;
}

cout<<KK<<" subcarriers transmitting"<<endl;
*/

   double Prand,PAPRran;
   getPower<complex<double>>(someVector,&Prand,&PAPRran);

   //cout<<"POWER randomVector(FREQ DOMAIN)= "<<Prand<<endl;
   vector<complex<double>> vectorIFFT;
   assignfft(someVector,TL,&vectorIFFT);
   //PowerVector(vectorIFFT,&P,&PAPR);
   //   cout<<" POWER : "<< P<< " PAPR : " <<PAPR<<endl;
   if(repeat){
       std::signal(SIGINT, &sig_int_handler);
       std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
   }

   vector<complex<double>> normalizedVector;
   normalized<complex<double>> (vectorIFFT, &normalizedVector,&Powerin,&Powerout,&PAPR);
   //PowerVector(normalizedVector,&P,&PAPR);

/*
 for(int i=0;i<vectorIFFT.size();i++){
 cout<<i<<") "<<vectorIFFT[i]<<endl;
}*/

 
    cout<<endl<<"POWER vectorIFFT: "<< Powerin<< " PAPR : " <<PAPR<<endl;
    cout<<"POWER normalized: "<< Powerout<< " PAPR : " <<PAPR<<endl<<endl;
  
   int powerdbm=stoi(pts[16]->value);
   double powerlinear=pow(10,(0.1*(powerdbm-30)));
   cout<<"POWER = "<<powerdbm<<" dBm"<<endl;
   cout<<"POWER = "<<powerlinear<<" watts"<<endl;


   for(int i=0;i<normalizedVector.size();i++){
   normalizedVector[i]*=sqrt(powerlinear);
   }
   double powerNorm,PAPRnorm;
   getPower<complex<double>>(normalizedVector,&powerNorm,&PAPRnorm);
   cout<<"POWER of norm vector times sqrt("<<powerlinear<<" watts) = "<<powerNorm<<" watts, and PAPR = "<<PAPRnorm<<endl;


/*

   for(int i=0;i<normalizedVector.size();i++){
   cout<<i<<") "<<normalizedVector[i]<<endl;
}
*/

   vector<complex<double>> vectorWindow;
   for(int i=0;i<Nwindow;i++){
   vectorWindow.insert(vectorWindow.begin(),vectorIFFT.begin(),vectorIFFT.end());
   }


   cout<<"vectorIFFT Size : "<<vectorIFFT.size()<<endl; 
   cout<<"vectorWindow size : "<<vectorWindow.size()<<endl;
   promise<void> exitSignal[sizeVec];   // future and promise for threads
   future<void> futureObj[sizeVec];
   thread threads[sizeVec];
   for (int i = 0; i < sizeVec; i++) {
   futureObj[i]=exitSignal[i].get_future();
   threads[i] = thread(&read_serialVectorMode,ports[i],move(futureObj[i]));}
   cout<<"Number of Threads started :"<<sizeVec<<string(50,' ')<<endl; 
   start_file = std::chrono::steady_clock::now();
   do{  
       //cout<<"\r \033[1;33mTRANSMITTING!\t\033[0m"<<flush<<endl;
   transmissionfunction(tx_stream,vectorWindow,spbVect);
   //cout<<"\r\033[F"<<flush;
   counter++;
   //cout<<endl;
   if(repeat and delay > 0.0) {
   this_thread::sleep_for(
   chrono::milliseconds(int64_t(delay*1000)));
   }
   } while(repeat and not stop_signal_called);
   finalMessage();
  // cout<<"\r\033[F"<<flush;
for(int i=0;i<sizeVec;i++){
   exitSignal[i].set_value();
   threads[i].join(); }
   cout<<"Number of Threads ended :"<<sizeVec<<string(50,' ')<<endl; 
   cout<<"Vector was transmitted : "<<counter<<" times"<<endl; 
   if (pts[13]->value.compare("ON")==0)  goto Iteration;
   else goto END_TX;


Iteration:
   cout<<"INSIDE Iteration label"<<endl;
   // goto TinVector;

END_TX:
   end_file = chrono::steady_clock::now();
   chrono::duration<double> diff_file = end_file-start_file;
   duration_file=diff_file.count();
   total_duration+=duration_file;
   checkTotalHarvested();
   //cout<<endl;
   //cout<<"\r\033[F"<<flush;

   return EXIT_SUCCESS;

}
//************************************************************************FUNCTIONS*************************************************************************************************************//
void transmissionfunction(uhd::tx_streamer::sptr tx_stream, vector<complex<double>> BUFF, size_t spb)
{
    uhd::tx_metadata_t md;
    md.start_of_burst = false ;
    md.end_of_burst = false; 
   int counter=0;
   //complex<double> *p;
   //complex<double> *end = &BUFF.back(); 
   //complex<double> *start= &BUFF[0];
   	//NEW_ERASE_LATER
   	//cout<<"BUFF.size() : "<<BUFF.size()<<endl;
   //	cout<<"END OF BUFF : "<<end<<endl;
   //	cout<<"START of BUFF : "<<start<<endl;
   //	cout<<"END-START : "<<end-start<<endl;
  
for(int i = 0 ;i<BUFF.size(); i +=spb){

	
  	signal(SIGINT, &sig_int_handler); 
 	
	if((BUFF.size()-i)<spb){
	spb=BUFF.size()-i;
	//cout<<i<<") SPB* :"<<spb<<endl;
        md.end_of_burst=true;        	
 	tx_stream->send(&BUFF.at(i),spb, md);
        counter++;
        //cout<<"counter = "<<counter<<endl;     	
	return;
        }

	//cout<<i<<") SPB :"<<spb<<endl;
        if(i==0)md.start_of_burst = true;
	if(i!=0)md.start_of_burst = false;
	tx_stream->send(&BUFF.at(i), spb, md);
        counter++;
	if (stop_signal_called==true) {//cout<<"\r\033[F"<<flush;
 	return;}
	}

}


 	
vector<complex<double>> randomVector(int totalN){
    vector<complex<double>>  randomVector;
    srand ( time(NULL) ); //just seed the generator
    for(int i=0;i<totalN;i++){
    complex<double> random_num;
    int real = rand()%40001 - 20000; //this produces numbers between -1000 - +1000
    int imag = rand()%40001 - 20000; //this produces numbers between -1000 - +1000
    double real_f=real/1000.0;//this will create random doubleing point numbers between -1 upto //1
    double imag_f=imag/1000.0;//this will create random doubleing point numbers between -1 upto //1
    random_num.real(real_f);  
    random_num.imag(imag_f);  
    randomVector.push_back(random_num);
    }
    return randomVector;
}

void assignfft( vector<complex<double>> randomVector, int NN, vector<complex<double>> *vectorIFFT){
    fftw_complex x[NN]; //This is equivalent to: double x[n][2];
    fftw_complex y[NN];//Output array
    for (int i = 0;i < NN; i++){//fill the first array with some data
    x[i][REAL] = real(randomVector[i]); //i.e.,{ 1, 2, 3, 4, 5}
    x[i][IMAG] = imag(randomVector[i]);
    fftw_cleanup();
    }
    ifft(x,y,NN );// compute the IFFT of x and store the results in y
    //cout << "\nIFFT =" << endl;// display the results
    complex<double> AUX1;
    vector<complex<double>> AUX2;
    for (int i = 0;i < NN; i++){//fill the first array with some data
    AUX1.real(y[i][REAL]); //i.e.,{ 1, 2, 3, 4, 5}
    AUX1.imag(y[i][IMAG]);
    fftw_cleanup(); 
    AUX2.push_back(AUX1);
    }
    *vectorIFFT=AUX2;
    vector<complex<double>> IFFT=AUX2;
    cout<<"SIZE OF AUX2 "<<AUX2.size()<<endl;
    // displayComplex(x,y,randomVector,IFFT);
}   

/* Displays complex numbers in the form a +/- bi. */
void displayComplex(fftw_complex *y,fftw_complex *x,vector<complex<double>> randomVector,vector<complex<double>> IFFT){   
    cout<<" RANDOM VECTOR \t FFT \t IFFT " << endl;
    for (int i = 0; i < randomVector.size(); ++i){
    cout <<randomVector[i];
    if (x[i][IMAG] < 0) cout << "\t\t"<<x[i][REAL] << " - " << abs(x[i][IMAG]) << "i" ;
    else cout << "\t\t"<<x[i][REAL] << " + " << x[i][IMAG] << "i";
    if (y[i][IMAG] < 0) cout <<"\t\t"<< y[i][REAL] << " - " << abs(y[i][IMAG]) << "i";
    else cout <<"\t\t"<< y[i][REAL] << " + " << y[i][IMAG] << "i";
    cout <<"\t\t"<<IFFT[i]<<endl;
    }
}
//FFT
void fft(fftw_complex *in, fftw_complex *out,int NN){
    fftw_plan plan = fftw_plan_dft_1d(NN, in, out, FFTW_FORWARD, FFTW_ESTIMATE);// create a DFT plan
    fftw_execute(plan);// execute the plan
    fftw_destroy_plan(plan);// do some cleaning
    fftw_cleanup();
}   
//IFFT 
void ifft(fftw_complex *in, fftw_complex *out,int NN){	
    fftw_plan plan = fftw_plan_dft_1d(NN, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);// create an IDFT plan
    fftw_execute(plan);// execute the plan
    fftw_destroy_plan(plan);// do some cleaning
    fftw_cleanup();
    for (int i = 0; i < NN; ++i){ 	// scale the output to obtain the exact inverse
    out[i][REAL] /= NN;
    out[i][IMAG] /= NN;}
}

void PowerVector(vector<complex<double>> inVector,double *Power,double *PAPR){
    size_t Ntotal= inVector.size();
    double mod,sum=0,max=0;
    for(int i=0;i<Ntotal;i++){
    //cout<<inVector[i]<<endl;
    mod=pow(real(inVector[i]),2)+pow(imag(inVector[i]),2);
    if(mod>max)max=mod;
    sum+=mod;
    }
    *PAPR=max/sum;
    *Power=sum/Ntotal;
}
/*
void normalized(vector<complex<double>> inVector,vector<complex<double>> *Yvector ){
    size_t Ntotal= inVector.size();
    double P,PAPR;
    PowerVector(inVector,&P,&PAPR);
    for(int i=0;i<Ntotal;i++){
    inVector[i]/=sqrt(P);
    }
    *Yvector=inVector;
}
*/
//Create Folders that contains Output samples and Csv files
void creatingFolders(){
    const int dir_err_2 = system("mkdir -p ../CSV_files");
    if (-1 == dir_err_2)
    {printf("Error creating directory!n");exit(1);}
    chrono::time_point<std::chrono::system_clock> time_now = std::chrono::system_clock::now();
    time_t time_now_t = std::chrono::system_clock::to_time_t(time_now);
    tm now_tm = *std::localtime(&time_now_t);
    strftime(DATE, 20, "%d%m%Y_%H_%M", &now_tm);
    string DATE_s(DATE);
    if(saveCSV){
    string SSS= "mkdir -p ../CSV_files/"+DATE_s;
    const char * c = SSS.c_str();
    const int dir_err_1 = system(c);
    if (-1 == dir_err_1)
    {printf("Error creating directory!n");exit(1);}
    }

    const int dir_err_3 = system("mkdir -p ../Sample_outputs");
    if (-1 == dir_err_3)
    {printf("Error creating directory!n");exit(1);}
    string TTT= "mkdir -p ../Sample_outputs/"+DATE_s;
    const char * d = TTT.c_str();
    const int dir_err_4 = system(d);
    if (-1 == dir_err_4)
    {printf("Error creating directory!n");exit(1);}
}
//COUNTDOWN BEFORE START!
void countdown(){
    for (int i=3;i>=0;i--){
    cout<<"\r\033[1;33mTRANSMISSION STARTS IN ----> "<<i<<"\033[0m"<<flush;
    this_thread::sleep_for (chrono::seconds(1));
    if (i==0) cout<<"\r"<<string(50,' ')<<flush;
    }
    cout<<endl<<endl;
}
//CHECK IF BUFF TO STOLD IS A NUMBER
bool is_number(const std::string& s){
    int ch=0;
    string::const_iterator it = s.begin();
    while (it != s.end() && isdigit(*it)){++it;++ch;}
    if (it <=s.end()) return false;
    if(it>s.end() || ch>19) return true;
}
//GET DIRECTORY LIST
int getdir (string dir, vector<string> &files){
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
    cout << "Error(" << errno << ") opening " << dir << endl;
    return errno;}
    while ((dirp = readdir(dp)) != NULL) {
    files.push_back(string(dirp->d_name));}
    closedir(dp);
    CheckDot(files);
    return 0;
}
//CHECK THE DOT AND ERASE, FOR READING FOLDER
void CheckDot(vector<string> &files){
    for(int i=0;i<files.size();i++){
    if(files[i]==".") files.erase(files.begin()+i);
    if(files[i]=="..") files.erase(files.begin()+i);  
    }
}	
//GET BYTE SIZE OF FILE
void map_file(char *fname, size_t *flen){   
    int input_fd;
    size_t inputlen;
    input_fd = open(fname, O_RDONLY, 0 /*used only on creation*/);
    if (input_fd < 0) {
    fprintf(stderr,"Cannot open '%s'\n",fname);
    goto end;}
    // Get the size of the file
    struct stat tmpstat;
    if ( 0 != fstat(input_fd, &tmpstat) ) {
    fprintf(stderr,"Cannot stat '%s'\n",fname);
    goto end;}
    inputlen = tmpstat.st_size;
    *flen = inputlen;
    end:
    close(input_fd);    
}
//FILE TO VECTOR 
/*
vector<complex<double>> makeVector( const std::string &file,int length){
    bool endOfFile=false;
    int size_of_vector= length/sizeof(complex<double>);
    cout<<"size_of_vector = "<<size_of_vector<<endl;
    std::vector<complex<double>> buff(size_of_vector);    
    cout<<"size_of_buff = "<< buff.size()<<endl;
    cout<<"length = "<<length<<endl;
    std::ifstream infile(file.c_str(), std::ifstream::binary);
    infile.read((char*)&buff.front(), length);
    infile.close();
    //for(int i=0;i<50;i++){cout<<"["<<i<<"] : "<<buff[i]<<endl;}
    return buff;
}
*/
//CHECK ARDUINO PORTS CONNECTED
vector<string> check_ports(int NN,int baudrate){
    vector<string> ports;
    char serial[256];
    for (int i=0;i<NN;i++){
    string aa="/dev/ttyUSB";
    string num= to_string(i);
    string serial_str=aa+num;
    //cout<< aa+num<<endl;
    strcpy (serial,serial_str.c_str());
    int fd=serialport_init(serial,baudrate);
    //this_thread::sleep_for (chrono::seconds(1)); 
    if(fd!=-1){ports.push_back(serial_str);}
    }
    return ports;
}
//CHECK NUMBER OF SAMPLES
void check_numberSamples(int Nsamples, string IDPH){
    for (int i=0;i<Nports;i++)	{
    if(IDS[i].compare(IDPH)==0) total_samples[i]=Nsamples;}
}
//CHECK DIFFERENCE BETWEEN FILE AND THREAD DURATION
void check_diff(void){
    //cout<<"duration_file = "<<duration_file<<" seconds"<<endl;
    for( int i=0;i<Nports;i++){
    if(duration_thread[i]!=0){
    difference_file_thread[i]= duration_file-duration_thread[i];}
    }
}
//SUMMARY
void writeSummary(string fileName,int N_check){
    fileName.erase(fileName.end()-4,fileName.end());
    string DATE_s(DATE);
    string file_name= "../Sample_outputs/"+DATE_s+"/"+fileName+".txt";
    ofstream myfile; 
    myfile.open (file_name);
    myfile<<"Summary file for "+fileName+" :\n\n";
    myfile<<"Total duration of file transmission\t\t\t: "<<set_time(duration_file)<<"\n";
    myfile<<"Total number of repetition\t\t\t\t: "<<N_check<<"\n\n";
    for(int i=0;i<Nports;i++){
    if(!IDS[i].empty()){
    myfile <<"**********************************************************************************\n";
    myfile << "*************************** ID "+IDS[i]+" ****************************************\n";
    myfile << "Total duration\t\t\t\t\t\t: "<<set_time(duration_thread[i])<<"\n";
    myfile << "Average Power harvested\t\t\t\t\t: "<<setprecision(18)<<Averages[i]<<" [mW]"<<"\n";
    myfile << "Diffence between file transmission and data reading \t: "<<difference_file_thread[i]<<" seconds"<<"\n";
    myfile << "Total Number of samples taken\t\t\t\t: "<< total_samples[i]<<" samples\n\n";
    }
    }
    myfile.close();
}
//CHECK TOTAL DURATION OF THE FILE
void checktime_file(double check_time){
    cout<<"Total time of file transmission: "<<set_time(check_time)<<endl;
}
//CHECK ID
int checkfot(string ID){
    int fot;
    for(int i=0;i<Nports;i++){
    if (IDS[i].compare(ID)==0)fot=i;
    }
    return fot;
}
//PRINT TOTAL THREAD TIME
void checktime_thread(void){
    for (int i=0;i<Nports;i++){
    if(!IDS[i].empty()){
    string t_f = IDS[i];
    cout<<"Total duration for thread with ID "+t_f+" : "<<set_time(duration_thread[i])<<endl;}	
    }
}
//SET TIME MEASURE
string set_time(double duration){
    string ss;
    if (duration>=60){duration/=60;
    if (duration>=60){duration/=60;ss= to_string(duration)+" hours";return ss;}
    ss = to_string(duration) + " minutes" ;return ss;}
    ss = to_string(duration) + " seconds" ;return ss; 
}
//PRINT TOTAL HARVESTED IN EVERY ARDUINO
void checkTotalHarvested(){
    for (int i=0;i<Nports;i++){
    if(Averages[i]!=0){cout<<"POWER HARVESTER "+IDS[i]+" harvested : "<<Averages[i]<<" mW in average"<<endl;
    cout<< "Total Number of samples ID "+IDS[i]+" : "<< total_samples[i]<<endl;}
    }
}
//CHANGE GLOBAL VARIABLE AVERAGE FOR SPECIFIC ID
void checkid(string ID,long double average_thread){
    for(int i=0;i<Nports;i++){
    if(IDS[i].compare(ID)==0)Averages[i]=average_thread;}
}
//PRINT "TRANSMISSION IS OVER"
void finalMessage(){
    cout<<"\033[1;32m\r"<<string(102,'*')<<"\033[0m"<<endl;
    cout<<  "\033[1;32m"<< string(25,'*')<<"[" <<string(15,'-')<<"TRANSMISSION IS OVER"<<string(15,'-')<<"]"<< string(25,'*')<<"\033[0m"<<endl;
    cout<<"\033[1;32m"<<string(102,'*')<<"\033[0m"<<endl<<endl;
    cout<<"TOTAL TRANSMISSION TIME : "<< set_time(total_duration)<<endl;
}
//SERIALPORT_INITIALIZATION//
int serialport_init(const char* serialport, int baud){
    struct termios toptions;
    int fd;
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)return -1;
    if (tcgetattr(fd, &toptions) < 0) {perror("init_serialport: Couldn't get term attributes");return -1;}
    speed_t brate = baud; // let you override switch below if needed
  /*  
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    */
    cfsetospeed(&toptions, brate);
    cfsetispeed(&toptions, brate);

    toptions.c_cflag &= ~PARENB;// 8N1
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS; // no flow control
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {perror("init_serialport: Couldn't set term attributes");return -1;}
    return fd;
}    
