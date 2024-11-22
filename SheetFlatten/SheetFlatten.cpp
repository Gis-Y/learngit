// SheetFlatten.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "ModelIO/ModelIO.h"
#include "SheetFlattenCore.h"

using namespace std;

int main()
{
	vector<Standard_Integer> SoltListId;
	vector<Standard_Integer> sliceListId;
	SheetFlattenCore* core = new SheetFlattenCore();
	core->m_SplitListId = sliceListId;
	core->m_SoltListId = SoltListId;
	TopoDS_Compound aCompound1;
	//FlattenFace flat;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aCompound1);
	if (core->Read("D:\\1_work\\1233333.stp"))
	{
		core->Perform();
		TopoDS_Edge baseEdge;
		core->m_process.generate(aCompound1);
		delete core;


		return 0;
	}


}
