c++11的资源管理机制 RAII(Resource Acquisition Is Initialization)机制

```c++
RAII是Resource Acquisition Is Initialization的缩写,它是C++中一种资源管理技术。
其核心思想是:将资源的获取和释放绑定在对象的构造和析构函数上,从而让资源的管理自动化。
一般来说,在C++中,我们可以使用RAII技术来管理如下资源:
- 内存:使用智能指针std::unique_ptr、std::shared_ptr自动管理内存,避免内存泄漏。
- 文件:使用std::fstream在作用域结束时自动关闭文件。
- Mutex:使用std::lock_guard自动上锁和解锁。
- 线程:使用std::thread自动调用join()等待线程结束。
RAII带来的主要优点是:
1. 简化代码,不需要显示调用资源的关闭/释放方法。
2. 避免资源泄漏,因为资源是绑定在作用域上,离开作用域会自动释放。
3. 即使有异常抛出,资源也会被正常释放。这避免了在函数退出时忘记资源释放导致的问题。
4. 资源获取和释放时机清晰,就是对象的构造和析构时刻。这使得程序的逻辑更加清晰。
---
在C++11中,std::lock_guard用于自动加锁和解锁mutex
template <class Mutex> 
class lock_guard {
public:
	explicit lock_guard(Mutex& m);// 构造函数加锁
	~lock_guard();// 析构函数解锁
	Mutex& mutex() { return m; }// 返回mutex
private:
	Mutex& m;
};
它在构造函数中对传入的mutex加锁,在析构函数中对mutex解锁。这确保在lock_guard实例生命周期内,它所持有的mutex一直处于加锁状态。所以它通常用于通过在函数作用域内实例化lock_guard,简单安全地加锁和解锁:
void foo() {
	std::mutex m;
	{
		std::lock_guard<std::mutex> lock(m);
		// do something...
	}// lock_guard析构,自动解锁m
}
这可以避免在函数退出时出现异常而忘记解锁mutex,造成死锁的问题。std::lock_guard运用了RAII技术,简化了对mutex的加锁/解锁操作,使线程同步变得更加简单高效。这是C++11中线程同步方面提供的非常有用的机制。
---
在C++11中,std::shared_ptr可以自动管理堆内存,避免内存泄漏。
std::shared_ptr有以下主要特点:
1. 它维护一个引用计数,记录正在使用该指针的对象数目。当引用计数为0时,自动删除被管理的对象。
2. 它可以由多个对象共享同一块内存。这是因为它有引用计数的机制。
3. 如果发生循环引用,std::shared_ptr可以正常工作。这是因为它使用弱引用(weak_ptr)来解决循环引用的问题。
4. 它具有RAII语义,离开作用域时会调用delete释放内存,不会造成内存泄漏。
void foo() {
	std::shared_ptr<int> ptr(new int(10));// ptr引用计数为1
	{
		std::shared_ptr<int> ptr2 = ptr;// ptr引用计数为2
    }// ptr2离开作用域,ptr引用计数为1
}// ptr离开作用域,引用计数为0,内存被释放
```

=delete

```c++
在C++中,=delete是函数的删除声明,表示该函数是被删除的,不能被调用。
当一个类中定义了=delete的函数,如果有代码尝试调用这个函数,编译器会报错。这个功能通常用于:
1. 防止某个函数被误调用。例如无意义或危险的操作。
2. 当一个函数不能由算法自动生成时(如默认构造函数),可以声明为=delete。
3. 当基类中定义的虚函数在派生类中不需要重写或不被允许重写时,基类可以将该虚函数声明为=delete。
格式为: 函数返回值类型 函数名(参数列表) = delete;
---
struct Foo {
	Foo() = delete;// 禁用默认构造函数
	void bar() = delete;// 禁用bar()函数
	virtual void val() = delete;// 禁用val()函数
};
int baz() = delete;// 全局函数baz()被删除
上述代码中:
- Foo类的默认构造函数被删除,不能实例化该类
- Foo类的bar()方法被删除,不能调用该方法
- Foo类的val()方法被删除,该类的派生类不能重写该方法
- 全局函数baz()被删除,不能调用该函数
---
注意:
当基类的默认构造函数被删除后,派生类是否还能有默认构造函数取决于以下3个因素,派生类:
1. 如果需要使用基类的默认构造函数初始化基类子对象,也需要删除自己的默认构造函数;
2. 如果通过其他方式初始化基类子对象,并且自身也需要默认构造函数,可以保留自己的默认构造函数;
3. 如果自身不需要默认构造函数,可以显式地删除自己的默认构造函数。
struct Base {
	Base() = delete;// 基类默认构造函数被删除
};
struct Derived1 : Base {
    // 编译错误:无法初始化基类子对象
    Derived1() {} 
};
struct Derived2 : Base {
    Derived2(int x) : Base({}) {}// 使用其他构造函数初始化基类子对象
    // 派生类可以有自己的默认构造函数
};
struct Derived3 : Base {
    Derived3() = delete;// 派生类也删除自己的默认构造函数(显式)
};
```

=default

```c++
在C++中,=default表示使用编译器自动生成的函数。它通常用于:
1. 析构函数:当类中没有定义析构函数时,编译器会自动生成一个默认析构函数。我们可以显式地声明=default,表示使用这个自动生成的析构函数。
2. 拷贝构造函数和赋值运算符:当类中没有定义这两个函数时,编译器也会自动生成默认的函数。我们可以用=default显式使用编译器自动生成的版本。
3. 其他特殊成员函数:如默认构造函数、移动构造函数和移动赋值运算符。
格式为: 函数返回值类型 函数名(参数列表) = default;
---
struct Foo {
  ~Foo() = default;// 使用自动生成的析构函数
  Foo(const Foo&) = default;// 使用自动生成的拷贝构造函数
  Foo& operator=(const Foo&) = default;// 使用自动生成的拷贝赋值运算符
};
上述代码中:
- 析构函数使用自动生成的版本
- 拷贝构造函数和赋值运算符也使用自动生成的版本
所以=default的作用是明确表示使用编译器自动生成的某个特殊成员函数。相比于自定义这些函数,=default的优点是:
1. 简单方便,不需要自己实现。
2. 编译器生成的版本一般比较高效,特别是移动构造和移动赋值。
3. 接口简洁,没有额外增加的函数。
```

explicit

```c++
在C++中,explicit关键字用于修饰构造函数,表示该构造函数是显式的,不发生隐式类型转换。
explicit关键字阻止隐式类型转换发生在构造函数上,要求使用显式调用语法来调用构造函数。它通常用于:
1. 避免意外的隐式类型转换导致的bug。
2. 构造函数的参数可以从其他类型转换得来,但转换后的语义并不明确。这时可以声明为explicit,强制programmer显式进行构造函数调用,表达清晰的语义。
---
注意,当基类的构造函数被声明为explicit时,派生类的构造函数一般也需要声明为explicit。主要有以下两种情况:
1. 派生类继承自基类,并在构造函数中显示地调用基类的构造函数。此时派生类的构造函数也需要声明为explicit,否则会发生从派生类类型隐式转换为基类类型的情况;
2. 派生类的构造函数可以从其他类型的参数转换得来,但转换后的语义并不明确。此时在派生类的构造函数上也声明explicit,强制程序员在实例化对象时显式调用构造函数,表达清晰的语义;
struct Base {
	explicit Base(int) {} 
};
struct Derived : Base {
	explicit Derived(double d) : Base(static_cast<int>(d)) {}
};
int main() {
	Derived d = 1.1;// 错误,Derived构造函数为explicit
	Derived d(1.1);// 正确,需要显式调用构造函数
}
```

