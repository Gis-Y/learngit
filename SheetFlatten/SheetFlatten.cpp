// SheetFlatten.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

//#include "ModelIO/ModelIO.h"
#include "SheetFlattenCore.h"

using namespace std;

int main()
{
	vector<Standard_Integer> SoltListId;
	vector<Standard_Integer> sliceListId;
	SheetFlattenCore* core = new SheetFlattenCore();
	core->setSplitListId(sliceListId);
	core->setSoltListId(SoltListId);

	if (core->Read("D:\\1_work\\standard.stp"))
	{
		core->Perform();
		delete core;
		return 0;
	}
	delete core;
	return 1;


}
