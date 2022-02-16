#include<iostream>
#include<string>
using namespace std;
class Base					//基类
{
public:
	virtual void TestFunc1()
	{
		cout << "Base::TestFunc1()" << endl;
	}
	virtual void TestFunc2()
	{
		cout << "Base::TestFunc2()" << endl;
	}
	virtual void TestFunc3()
	{
		cout << "Base::TestFunc3()" << endl;
	}
	int _b;
};
class Derived : public Base					//派生类
{
public:
	virtual void TestFunc4()
	{
		cout << "Derived::TestFunc4()" << endl;
	}
	virtual void TestFunc1()
	{
		cout << "Derived::TestFunc1()" << endl;
	}

	virtual void TestFunc3()
	{
		cout << "Derived::TestFunc3()" << endl;
	}
	virtual void TestFunc5()
	{
		cout << "Derived::TestFunc5()" << endl;
	}
	int _d;
};
typedef void(*PVFT)();				//声明函数指针，用来调用虚函数

void PrintVFT(Base& b, const string& str)
{
	cout << str << endl;
	/*这里是先将对象的地址取出来再强制类型换，此时再解引用的话，取的值就是对象前四个字节地址里面存放的值，
	这个值就是虚表的地址，即就是函数指针数组的首地址，我们再将这个地址转换成函数指针类型*/
	PVFT* pVFT = (PVFT*)(*(int*)&b);
		
	while (*pVFT)					//虚表中最后一个元素是空值，打印完循环退出
	{
		(*pVFT)();			// 再解引用就是函数指针数组里面的第一个元素(即就是第一个虚函数地址),再往后一次打印
		++pVFT;
	}
	cout << endl;
}
void TestVirtualFunc(Base& b)
{
	b.TestFunc1();
	b.TestFunc3();
	return;
}
int main()
{
	Base b;
	Derived d;
	// 打印基类与派生类的虚表
	PrintVFT(b, "Base VFT:");
	PrintVFT(d, "Derived VFT:");
	// 传递Base类对象
	TestVirtualFunc(b);
	cout << endl;
	// 传递派生类对象
	TestVirtualFunc(d);
	system("pause");
	return 0;
}
