// SheetFlatten.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "ModelIO/ModelIO.h"
#include "SheetFlattenCore.h"

using namespace std;

int main()
{
	/*std::vector<std::tuple<point_t, point_t, std::string>> ai;
	std::vector<std::tuple<point_t, double, std::string>> ai1;
	std::vector< std::tuple< point_t, double, double, double, std::string > > ai2;*/

	SheetFlattenCore* core = new SheetFlattenCore();
	TopoDS_Compound aCompound1,aCompound2;
	FlattenFace flat;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aCompound1);
	aBuilder.MakeCompound(aCompound2);
	if (core->Read("D:\\1_work\\1233333.stp"))
	{
		core->Perform();
		TopoDS_Edge baseEdge;
		core->aFalttenMove.generate(aCompound2);

		for (auto it : core->aFalttenMove.m_ThreeToTwoEdge)
		{
			TopoDS_Edge edge;
			vector<TopoDS_Edge> aEdges;
			edge = it.second;
			if (core->aFalttenMove.m_oldMapNewEdge.find(edge) != core->aFalttenMove.m_oldMapNewEdge.end())
			{
				aEdges = core->aFalttenMove.m_oldMapNewEdge.find(edge)->second;
				for (auto elem : aEdges)
				{
					aBuilder.Add(aCompound1, elem);
				}
			}
			else
			{
				aBuilder.Add(aCompound1, edge);
			}
		}
		FlattenFace::BuildDocStd(aCompound1);
		delete core;


		return 0;
	}


}
