#include <boost/thread.hpp>
#include <iostream>

using namespace std;

void ThreadFunction()
{
    int counter = 0;

    while(counter < 20)
    {
        cout << "thread iteration " << ++counter << " Press Enter to stop" << endl;

        try
        {
            // Sleep and check for interrupt.
            // To check for interrupt without sleep,
            // use boost::this_thread::interruption_point()
            // which also throws boost::thread_interrupted
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        catch(boost::thread_interrupted&)
        {
            cout << "Thread is stopped" << endl;
            return;
        }
    }
}

int main()
{
    // Start thread
    //boost::thread t(&ThreadFunction);
    boost::thread t = boost::thread(&ThreadFunction); 
    // Wait for Enter 
    char ch;
    cin.get(ch);

    // Ask thread to stop
//    t.interrupt();

    // Join - wait when thread actually exits
   // t.join();
    cout << "main: thread ended" << endl;

    return 0;
}

/*
class HelloWorld
{
public:
 void hello()
 {
   while (true)
	{
	try
	{


    std::cout <<
    "Hello world, I''m a thread!"
    << std::endl;
//	boost::this_thread::sleep(boost::posix_time::milliseconds(50));
}
catch(boost::thread_interrupted&)
        {
            cout << "Thread is stopped" << endl;
            return;
        }


 }
}
 void start()
 {
 // boost::function0< void> f =  boost::bind(&HelloWorld::hello,this);
 // boost::thread thrd( f );
	thrd = boost::thread(&HelloWorld::hello,this);
 // thrd.join();
 }
 boost::thread thrd;
 
}; 
int main(int argc, char* argv[])
{
 HelloWorld hello;
 hello.start();
char ch;
    cin.get(ch);

    // Ask thread to stop
    hello.thrd.interrupt();
 //   usleep(50*1000);  	   
// hello.thrd.join();
 return 0;  */
//}
