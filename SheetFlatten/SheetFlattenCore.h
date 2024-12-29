#ifndef SHEETFLATTENCORE_H
#define SHEETFLATTENCORE_H

#include <GeomAbs_SurfaceType.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
//#include <GeomConvert_SurfToAnaSurf.hxx>
//#include <GeomConvert_CurveToAnaCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <GeomLib.hxx>
#include <GeomAPI.hxx>
#include <TopoDS.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <IntTools_EdgeEdge.hxx>
#include <GeomLProp_SLProps.hxx>
#include <ProjLib.hxx>
#include <IntAna2d_AnaIntersection.hxx>


#include "ModelIO/ModelIO.h"
#include "tcl_5.0.6/tree.h"
#include "SheetFlattenBase.h"
#include "SheetFlattenFitting.h"
#include"SheetFlattenEdgeData.h"
#include"SheetFlattenProcess.h"

#include <iostream>
#include <vector>
using namespace std;
using namespace tcl;


class SheetFlattenCore
{
public:
	SheetFlattenCore()
	{
		aMeshParams.Deflection = 0.02;
		aMeshParams.Angle = 0.1;
		aMeshParams.Relative = Standard_True;
		aMeshParams.InParallel = Standard_True;
		aMeshParams.MinSize = Precision::Confusion();
		aMeshParams.InternalVerticesMode = Standard_True;
		aMeshParams.ControlSurfaceDeflection = Standard_True;
		aMeshParams.MeshAlgo = IMeshTools_MeshAlgoType_Delabella;

		//CurveDeflection = 0.001;
		//AngularDeflection = 1 / M_PI_4;

		faceid = 0;
		refId = 0;
	}
public:
	bool Read(const string& filePath);
	void setdoc(const Handle(TDocStd_Document)&document);
	int Perform();

	double GetFacesAngle(const TopoDS_Edge& edge);
	void setSoltEdges(const set<TopoDS_Edge>& edges);
	void setSplitEdges(const set<TopoDS_Edge>& edges);
	void setSoltListId(const vector<Standard_Integer>& data);
	void setSplitListId(const vector<Standard_Integer>& data);

private:

	void BuildEdgeAdjFacesMap(const TopoDS_Shape &objShape);
	void BuildFacesAngleMap();
	bool BuildFlattenTree(const TopoDS_Shape& objShape);
	void process();
	bool isFacePlane(const TopoDS_Face& face);
	void BuildFlattenTreeNode(tree<FlattenFaceNode>::iterator it);
	void Jigsaw(tree<FlattenFaceNode>::iterator fatherIt);
	void Fitting(tree<FlattenFaceNode>::iterator it);

	void DumpTree(tree<FlattenFaceNode>::pre_order_iterator it);
	void DumpTree(tree<FlattenFaceNode>::level_order_iterator it);
	void DumpTree(tree<FlattenFaceNode>::iterator it);

	bool CalculateAngleBetweenFaces(const TopoDS_Face& faceBase, const TopoDS_Face& faceTarget, double &angle);
	//gp_Pnt ProjectPointToPlane(const gp_Pnt& point, const gp_Pln& plane);

private:
	Handle(TDocStd_Document) doc;
	TopoDS_Compound aCompound;
	BRep_Builder aBuilder;
	IMeshTools_Parameters aMeshParams;

	FlattenFaceNode rootNode;		// 这里定义这个变量是为了对应指定基准面展开时，交互定义
	vector<FlattenFaceNode> m_vector_rootNodes;
	tree< FlattenFaceNode> shapeTree;
	vector< tree< FlattenFaceNode>> m_vector_shapTrees;
	vector<FlattenFace> aFlattenFaces;

	FlattenFace m_flat;
	TopTools_IndexedMapOfShape faceMap;
	map<TopoDS_Edge, pair<int, int>> m_EdgeToEdgeDataIndex;
	map<TopoDS_Edge, pair<int, int>> m_3dEdgeToEdgeDataIndex;

private:
	SheetFlattenProcess  m_process;
	map<TopoDS_Edge, pair<TopoDS_Edge, CurveType>> m_m3dTo2d;
	vector<vector<SheetFlattenEdgeData>> m_vEdgeDatas;
	vector<SheetFlattenEdgeData> m_EdgeDatas;
	SheetFlattenEdgeData m_EdgeData;
private:
	vector<Standard_Integer> m_SoltListId;
	vector<Standard_Integer> m_SplitListId;
	set<TopoDS_Edge> m_setSplitEdges;
	set<TopoDS_Edge> m_setSoltEdges;
	map<TopoDS_Edge, set<TopoDS_Face> > mapEdgeAdjFaces;
	map<TopoDS_Edge, double> mapFacesAngle;
	set<TopoDS_Face> finishedFaces;
	vector<TopoDS_Face> m_vector_AllFaces;
	int faceid;
	int refId;

};

void SheetFlattenCore::setSoltListId(const vector<Standard_Integer>& data)
{
	m_SoltListId = data;
}

void SheetFlattenCore::setSplitListId(const vector<Standard_Integer>& data)
{
	m_SplitListId = data;
}


inline bool SheetFlattenCore::Read(const string& filePath)
{
	ModelIO* modelIO = new ModelIO();
	if (modelIO->Read(filePath))
	{
		doc = modelIO->GetDoc();
		delete modelIO;
		return true;
	}
	else
	{
		delete modelIO;
		return false;
	}
}

void SheetFlattenCore::setdoc(const Handle(TDocStd_Document)& document)
{
	doc = document;
}

void SheetFlattenCore::setSoltEdges(const set<TopoDS_Edge>& edges)
{
	m_setSoltEdges = edges;
}

void SheetFlattenCore::setSplitEdges(const set<TopoDS_Edge>& edges)
{
	m_setSplitEdges = edges;
}

inline int SheetFlattenCore::Perform()
{
	Handle(XCAFDoc_ShapeTool)ST = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

	TDF_LabelSequence Labels;
	ST->GetFreeShapes(Labels);

	aBuilder.MakeCompound(aCompound);

	for (int idx = 1; idx <= Labels.Length(); ++idx)
	{
		TDF_Label theLabel = Labels.ChangeValue(idx);
		TopoDS_Shape objShape;
		ST->GetShape(theLabel, objShape);		

		BRepMesh_IncrementalMesh(objShape, aMeshParams);

		BuildEdgeAdjFacesMap(objShape);
		BuildFacesAngleMap();

		if (!BuildFlattenTree(objShape))
		{
			return 1;	// 没有root face
		}
		else {
			process();
		}
	}

	m_process.generate();
	return 0;
}
inline double SheetFlattenCore::GetFacesAngle(const TopoDS_Edge& edge)
{
	return mapFacesAngle[edge];
}
inline void SheetFlattenCore::BuildEdgeAdjFacesMap(const TopoDS_Shape &objShape)
{
	for (TopExp_Explorer faceExp(objShape, TopAbs_FACE); faceExp.More(); faceExp.Next())
	{
		//faceMap.Add(faceExp.Current());
		const TopoDS_Face& objFace = TopoDS::Face(faceExp.Current());
		m_vector_AllFaces.emplace_back(objFace);
		for (TopExp_Explorer edgeExp(objFace, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			faceMap.Add(edgeExp.Current());
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());
			set<TopoDS_Face> aFaces;
			if (mapEdgeAdjFaces.find(objEdge) != mapEdgeAdjFaces.end())
			{
				mapEdgeAdjFaces[objEdge].insert(objFace);
			}
			else
			{
				//set<TopoDS_Face> aFaces;
				aFaces.insert(objFace);
				mapEdgeAdjFaces[objEdge] = aFaces;
			}
		}
	}
}
inline void SheetFlattenCore::BuildFacesAngleMap()
{
	for (auto item : mapEdgeAdjFaces)
	{
		if (item.second.size() != 2)
		{
			continue;
		}
		double angle;

		set<TopoDS_Face>::iterator iter = item.second.begin();

		TopoDS_Face faceBase = *iter;
		iter++;

		TopoDS_Face faceTarget = *iter;

		if (CalculateAngleBetweenFaces(faceBase, faceTarget, angle))
		{
			mapFacesAngle.insert(make_pair(item.first, angle));
		}
	}
}
// 函数计算两面夹角
inline bool SheetFlattenCore::CalculateAngleBetweenFaces(const TopoDS_Face& faceBase, const TopoDS_Face& faceTarget, double &angle) 
{
	// 计算面积是为了调试时用面积区分面
	//GProp_GProps System;
	//BRepGProp::SurfaceProperties(faceBase, System);
	//double faceBaseArea = System.Mass();
	//BRepGProp::SurfaceProperties(faceTarget, System);
	//double faceTargetArea = System.Mass();

	//cout << "Area: " << faceBaseArea << "  " << faceTargetArea << endl;

	// 获取第一个面的法向量
	Handle(Geom_Surface) surfaceBase = BRep_Tool::Surface(faceBase);
	GeomLProp_SLProps propsBase(surfaceBase, 0.5, 0.5, 1, Precision::Confusion());
	if (!propsBase.IsNormalDefined()) {
		std::cerr << "法向量未定义" << std::endl;
		return false;
	}

	gp_Dir normalBase = propsBase.Normal();
	normalBase = -normalBase;

	// 获取第二个面的法向量
	Handle(Geom_Surface) surfaceTarget = BRep_Tool::Surface(faceTarget);
	GeomLProp_SLProps propsTarget(surfaceTarget, 0.5, 0.5, 1, Precision::Confusion());
	if (!propsTarget.IsNormalDefined()) {
		std::cerr << "法向量未定义" << std::endl;
		return false;
	}
	gp_Dir normalTarget = propsTarget.Normal();	
	normalTarget = -normalTarget;
	if (normalTarget.IsEqual(normalBase, 1e-6))
	{
		angle = 0.;
		return true;
	}
	gp_Dir crossNormal = normalBase.Crossed(normalTarget);
	if (gp_Vec(crossNormal).Magnitude() < gp::Resolution()) 
	{
		return false;
	}

	// 计算夹角（弧度）
	angle = normalBase.Angle(normalTarget);

	// 判断两个Face是正折还是反折
	
	// 计算形心，创建投影平面plane
	GProp_GProps gpropsBase;
	BRepGProp::SurfaceProperties(faceBase, gpropsBase);
	gp_Pnt centerBase = gpropsBase.CentreOfMass();	

	GProp_GProps gpropsTarget;
	BRepGProp::SurfaceProperties(faceTarget, gpropsTarget);
	gp_Pnt centerTarget = gpropsTarget.CentreOfMass();

	gp_Pln plane(centerBase, crossNormal);

	//按照法线方向将形心平移，定义一个新的点
	// 设定平移距离
	Standard_Real distance = 100.0;

	gp_Vec transVecBase = distance * gp_Vec(normalBase);
	gp_Pnt centerBaseTrs = centerBase.Translated(transVecBase);

	gp_Vec transVecTarget = distance * gp_Vec(normalTarget);
	gp_Pnt centerTargetTrs = centerTarget.Translated(transVecTarget);

	//将四个点投影到plane上
	gp_Pnt2d centerBasePrj = ProjLib::Project(plane, centerBase);
	gp_Pnt2d centerBaseTrsPrj = ProjLib::Project(plane, centerBaseTrs);
	gp_Pnt2d centerTargetPrj = ProjLib::Project(plane, centerTarget);
	gp_Pnt2d centerTargetTrsPrj = ProjLib::Project(plane, centerTargetTrs);

	gp_Lin2d lineBasePrj(centerBasePrj, gp_Vec2d(centerBasePrj, centerBaseTrsPrj));
	gp_Lin2d lineTargetPrj(centerTargetPrj, gp_Vec2d(centerTargetPrj, centerTargetTrsPrj));

	IntAna2d_AnaIntersection Int2d(lineBasePrj, lineTargetPrj);
	if (!Int2d.IsDone()) 
	{ 
		return false; 
	}

	int nbpts = Int2d.NbPoints();

	gp_Pnt2d intersectionPnt(Int2d.Point(1).Value());

	gp_Vec2d vecBase2d(centerBasePrj, centerBaseTrsPrj);
	gp_Vec2d vecInt2d(centerBasePrj, intersectionPnt);

	double angle2d = vecBase2d.Angle(vecInt2d);

	if (fabs(angle2d) < 1e-6)
	{
		angle = angle + M_PI;
		//angle = angle * 180.0 / M_PI + 180;
	}

	//cout << angle * 180.0 / M_PI << endl;

	return true;
}

void SheetFlattenCore::process()
{
	int i = -1, j = -1;
	for (auto elem : m_vector_shapTrees)
	{
		++i;
		Jigsaw(elem.begin());

		Fitting(elem.begin());
		for (auto elem : m_m3dTo2d)
		{
			++j;
			m_EdgeData.setEdge_3d(elem.first);
			if (mapEdgeAdjFaces.find(elem.first) != mapEdgeAdjFaces.end())
			{
				vector<TopoDS_Face> aFaces(mapEdgeAdjFaces.find(elem.first)->second.begin(), mapEdgeAdjFaces.find(elem.first)->second.end());
				if (aFaces.size() >= 2)
				{
					m_EdgeData.setBendEdge(true);
				}
				m_EdgeData.setVector_face(aFaces);
			}
			if (mapFacesAngle.find(elem.first) != mapFacesAngle.end())
			{
				double angle = mapFacesAngle.find(elem.first)->second;
				m_EdgeData.setAngle(angle);
				if (angle < M_PI)
				{
					m_EdgeData.setNegativeBend(true);
				}
			}
			if (m_setSoltEdges.find(elem.first) != m_setSoltEdges.end())
			{
				m_EdgeData.setSoltEdge(true);
			}
			if (m_setSplitEdges.find(elem.first) != m_setSplitEdges.end())
			{
				m_EdgeData.setSplitEdge(true);
			}
			m_EdgeData.setOldEdge_2d(elem.second.first);
			m_EdgeData.SetCurveType(elem.second.second);
			if (elem.second.second != LineCurve)
			{
				m_EdgeData.setBendEdge(false);
			}
			m_EdgeDatas.emplace_back(m_EdgeData);
			m_EdgeData.clear();
			m_EdgeToEdgeDataIndex[elem.second.first] = make_pair(i, j);
			m_3dEdgeToEdgeDataIndex[elem.first] = make_pair(i, j);
		}
		j = -1;
		m_vEdgeDatas.emplace_back(m_EdgeDatas);
		m_EdgeDatas.clear();
		m_m3dTo2d.clear();
	}

	m_process.set3dEdgeIndex(m_3dEdgeToEdgeDataIndex);
	m_process.setEdgeData(m_vEdgeDatas);
	m_process.setOldEdgeIndex(m_EdgeToEdgeDataIndex);
	SaveDxfFile saveFile;
	saveFile.SaveDxf(m_flat.ai, m_flat.ai1, m_flat.ai2, "D:\\1_work\\test.dxf");
}

inline bool SheetFlattenCore::BuildFlattenTree(const TopoDS_Shape& objShape)
{
	cout << "BuildFlattenTree..." << endl;

	if (m_vector_AllFaces.size() > finishedFaces.size())
	{
		FlattenFaceNode aRootNode;
		tree<FlattenFaceNode> aShapeTree;
		m_vector_shapTrees.emplace_back(aShapeTree);
		m_vector_rootNodes.emplace_back(aRootNode);
		tree<FlattenFaceNode>& aShapeTree_ref = m_vector_shapTrees.back();
		FlattenFaceNode& aRootNode_ref = m_vector_rootNodes.back();

		// 如果没有指定root face，则找出面积最大的平面
		if (aRootNode_ref.isNull())
		{
			// 找到面积最大的平面作为root face
			double area = -1e6;
			TopoDS_Face maxAreaFace;
			for (TopExp_Explorer faceExp(objShape, TopAbs_FACE); faceExp.More(); faceExp.Next())
			{
				const TopoDS_Face& objFace = TopoDS::Face(faceExp.Current());
				if (finishedFaces.find(objFace) != finishedFaces.end())
				{
					continue;
				}
				if (!isFacePlane(objFace))
				{
					continue;
				}

				// 定义变量来存储属性
				GProp_GProps props;
				// 计算面的属性
				BRepGProp::SurfaceProperties(objFace, props);
				// 获取面的面积

				if (fabs(props.Mass()) > area)
				{
					maxAreaFace = objFace;
					area = fabs(props.Mass());
				}
			}

			aRootNode_ref.flattenFace.Init(maxAreaFace);
		}

		// 如果没有平面
		if (aRootNode_ref.isNull())
		{
			return false;
		}
		else
		{
			aRootNode_ref.flattenFace.OneStepFlattenFace();

			//rootNode.flattenFace.DumpNasEx("F:\\FaceInit_" + to_string(faceid) + ".nas", 0);

			string nasFile = "D:\\FaceOnestep_" + to_string(faceid++) + ".nas";

			//rootNode.flattenFace.DumpNasEx(nasFile, 1);
		}

		aRootNode_ref.id = ++refId;
		tree<FlattenFaceNode>::iterator it = aShapeTree_ref.insert(aRootNode_ref);
		finishedFaces.insert(aRootNode_ref.flattenFace.face);

		BuildFlattenTreeNode(it);
		BuildFlattenTree(objShape);
		
	}
	/*int i = -1, j = -1;
	for(auto elem :m_vector_shapTrees)
	{
		++i;
		Jigsaw(elem.begin());

		Fitting(elem.begin());
		for (auto elem : m_m3dTo2d)
		{
			++j;
			m_EdgeData.setEdge_3d(elem.first);
			if (mapEdgeAdjFaces.find(elem.first) != mapEdgeAdjFaces.end())
			{
				vector<TopoDS_Face> aFaces(mapEdgeAdjFaces.find(elem.first)->second.begin(), mapEdgeAdjFaces.find(elem.first)->second.end());
				m_EdgeData.setVector_face(aFaces);
			}
			if (mapFacesAngle.find(elem.first) != mapFacesAngle.end())
			{
				double angle = mapFacesAngle.find(elem.first)->second;
				m_EdgeData.setAngle(angle);
				if (angle < M_PI)
				{
					m_EdgeData.setPositiveBend(true);
				}
			}
			if (mapFacesAngle.find(elem.first) != mapFacesAngle.end())
			{
				m_EdgeData.setAngle(mapFacesAngle.find(elem.first)->second);
			}
			if (m_mapSoltEdges.find(elem.first) != m_mapSoltEdges.end())
			{
				m_EdgeData.setSoltEdge(true);
			}
			if (m_mapSplitEdges.find(elem.first) != m_mapSplitEdges.end())
			{
				m_EdgeData.setSplitEdge(true);
			}
			m_EdgeData.setOldEdge_2d(elem.second);
			m_EdgeDatas.emplace_back(m_EdgeData);
			m_EdgeData.clear();
			m_EdgeToEdgeDataIndex[elem.second] = make_pair(i, j);
			m_3dEdgeToEdgeDataIndex[elem.first] = make_pair(i, j);
		}
		j = -1;
		m_vEdgeDatas.emplace_back(m_EdgeDatas);
		m_EdgeDatas.clear();
		m_m3dTo2d.clear();
	} 

	m_process.setEdgeData(m_vEdgeDatas);
	m_process.setOldEdgeIndex(m_EdgeToEdgeDataIndex);
	SaveDxfFile saveFile;
	saveFile.SaveDxf(m_flat.ai, m_flat.ai1, m_flat.ai2, "D:\\1_work\\test.dxf");*/
	// 如果没有指定root face，则找出面积最大的平面
	/*
	if (rootNode.isNull())
	{
		// 找到面积最大的平面作为root face
		double area = -1e6;
		TopoDS_Face maxAreaFace;
		for (TopExp_Explorer faceExp(objShape, TopAbs_FACE); faceExp.More(); faceExp.Next())
		{
			const TopoDS_Face& objFace = TopoDS::Face(faceExp.Current());

			if (!isFacePlane(objFace))
			{
				continue;
			}

			// 定义变量来存储属性
			GProp_GProps props;
			// 计算面的属性
			BRepGProp::SurfaceProperties(objFace, props);
			// 获取面的面积

			if (fabs(props.Mass()) > area)
			{
				maxAreaFace = objFace;
				area = fabs(props.Mass());
			}
		}

		rootNode.flattenFace.Init(maxAreaFace);
	}

	// 如果没有平面
	if (rootNode.isNull())
	{
		return false;
	}
	else
	{
		rootNode.flattenFace.OneStepFlattenFace();

		//rootNode.flattenFace.DumpNasEx("F:\\FaceInit_" + to_string(faceid) + ".nas", 0);

		string nasFile = "E:\\FaceOnestep_" + to_string(faceid++) + ".nas";
		
		//rootNode.flattenFace.DumpNasEx(nasFile, 1);
	}

	rootNode.id = ++refId;
	tree<FlattenFaceNode>::iterator it = shapeTree.insert(rootNode);	
	finishedFaces.insert(rootNode.flattenFace.face);

	BuildFlattenTreeNode(it);

	//cout << "finishedFaces hash: " << endl;
	//for (set<TopoDS_Face>::iterator iter = finishedFaces.begin(); iter != finishedFaces.end(); iter++)
	//{
	//	cout << hash<TopoDS_Face>{}(*iter) << endl;
	//}

	//for (typename tree<FlattenFaceNode>::pre_order_iterator pit = shapeTree.pre_order_begin(); pit != shapeTree.pre_order_end(); pit++)
	//{
	//	cout << pit.node()->get()->id << endl;
	//}

	Jigsaw(it);

	Fitting(it);
	*/
	return true;
}


inline void SheetFlattenCore::BuildFlattenTreeNode(tree<FlattenFaceNode>::iterator it)
{
	vector<tree<FlattenFaceNode>::iterator> firstRound;

	for (TopExp_Explorer edgeExp(it.node()->get()->flattenFace.face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
	{
		const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

		for (set<TopoDS_Face>::iterator faceIt = mapEdgeAdjFaces[objEdge].begin(); faceIt != mapEdgeAdjFaces[objEdge].end(); faceIt++)
		{
			if (m_setSplitEdges.find(objEdge) != m_setSplitEdges.end())
			{
				continue;
			}
			// 如果边只有一个相邻面则不是公共边
			if (mapEdgeAdjFaces[objEdge].size() == 1)
			{
				continue;
			}
			// 一个边只包含两个相邻面所以，只需要判断是否是已展开的面即可，不许对比是不是与faceIt相同
			if (finishedFaces.find(*faceIt) != finishedFaces.end())
			{
				continue;
			}

			FlattenFaceNode _node;
			_node.flattenFace.Init(*faceIt);
			_node.edge = objEdge;

			//_node.flattenFace.DumpNasEx("F:\\FaceInit_" + to_string(faceid) + ".nas", 0);
			_node.flattenFace.OneStepFlattenFace();
			string nasFile = "D:\\FaceOnestep_" + to_string(faceid++) + ".nas";
			//_node.flattenFace.DumpNasEx(nasFile, 1);
			_node.id = ++refId;
			finishedFaces.insert(*faceIt);

			tree<FlattenFaceNode>::iterator child_it = it.node()->insert(_node);
			firstRound.push_back(child_it);
		}
	}

	for (int i = 0; i < firstRound.size(); i++)
	{
		BuildFlattenTreeNode(firstRound[i]);
	}
}
//inline void SheetFlattenCore::Jigsaw(tree<FlattenFaceNode>* faceNode)
inline void SheetFlattenCore::Jigsaw(tree<FlattenFaceNode>::iterator fatherIt)
{	

	faceid = 0;

	// 遍历子节点
	typename tree<FlattenFaceNode>::level_order_iterator lit = fatherIt.node()->level_order_begin();
	lit.node()->get()->flattenFace.Transform2ZPlane();
	//lit.node()->get()->flattenFace.DumpNasEx("F:\\Face2ZPlane_" + to_string(faceid) + ".nas", 2);
	//lit.node()->get()->flattenFace.DumpNasEx("E:\\FaceJigsaw_" + to_string(faceid++) + ".nas", 3);
	lit++;
	for ( ;lit != fatherIt.node()->level_order_end(); lit++)
	{
		FlattenFaceNode* fatherFlattenNode = lit.node()->parent()->get();
		fatherFlattenNode->ParseNode(lit.node()->get()->edge);
		lit.node()->get()->flattenFace.Transform2ZPlane(lit.node()->get()->edge, fatherFlattenNode->StartNode(), fatherFlattenNode->EndNode(), fatherFlattenNode->GetPlaneOrientation());
		//lit.node()->get()->flattenFace.DumpNasEx("F:\\Face2ZPlane_" + to_string(faceid) + ".nas", 2);

		lit.node()->get()->flattenFace.Jigsaw(fatherFlattenNode->StartNode2D(), fatherFlattenNode->EndNode2D());

		string nasFile = "D:\\FaceJigsaw_" + to_string(faceid++) + ".nas";

		//lit.node()->get()->flattenFace.DumpNasEx(nasFile, 3);
	}
}

inline void SheetFlattenCore::Fitting(tree<FlattenFaceNode>::iterator it)
{
	TopoDS_Compound aCompound;
	FlattenFace flat;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aCompound);

	for (typename tree<FlattenFaceNode>::level_order_iterator lit = it.node()->level_order_begin(); lit != it.node()->level_order_end(); lit++)
	{
		cout << "CreateOutline" << endl;

		//lit.node()->get()->flattenFace.CreateOutline(aCompound,m_flat, aFalttenMove.m_ThreeToTwoEdge);
		lit.node()->get()->flattenFace.CreateOutline(aCompound, m_flat, m_m3dTo2d);
		// aBuilder.Add(aCompound, lit.node()->get()->flattenFace.CreateOutline());
	}

	//FlattenFace::BuildDocStd(aCompound);
}

inline bool SheetFlattenCore::isFacePlane(const TopoDS_Face& face) 
{
	// 获取面的几何表示
	const Handle(Geom_Surface)& surface = BRep_Tool::Surface(face);
	// 使用GeomLib_IsPlanarSurface来判断是否是平面
	GeomLib_IsPlanarSurface ISPlanarSurface(surface);

	if (ISPlanarSurface.IsPlanar()) {
		// 如果是平面，则P是该平面的一点，N是法向量，Dist是原点距离平面的距离
		return true;
	}
	return false;
}
inline void SheetFlattenCore::DumpTree(tree<FlattenFaceNode>::iterator it)
{
	cout << "DumpTree id: " << it.node()->get()->id << endl;

	it.node()->get()->InitHash();
	cout << "edge: " << endl << it.node()->get()->edgeHash << endl;
	cout << "edge address: " << &(it.node()->get()->edge) << endl;
	cout << "face: " << endl << it.node()->get()->faceHash << endl;

	cout << "Face Edge: " << endl;

	for (int i = 0; i < it.node()->get()->faceEdgeHash.size(); i++)
	{
		cout << it.node()->get()->faceEdgeHash[i] << endl;
	}

	cout << endl;

}

inline void SheetFlattenCore::DumpTree(tree<FlattenFaceNode>::pre_order_iterator it)
{
	cout << "DumpTree id: " << it.node()->get()->id << endl;;

	it.node()->get()->InitHash();
	cout << "edge: " << endl << it.node()->get()->edgeHash << endl;
	cout << "edge address: " << &(it.node()->get()->edge) << endl;
	cout << "face: " << endl << it.node()->get()->faceHash << endl;
	
	cout << "Face Edge: " << endl;

	for (int i = 0; i < it.node()->get()->faceEdgeHash.size(); i++)
	{
		cout << it.node()->get()->faceEdgeHash[i] << endl;
	}

	cout << endl;
}

inline void SheetFlattenCore::DumpTree(tree<FlattenFaceNode>::level_order_iterator it)
{
	cout << "DumpTree id: " << it.node()->get()->id << endl;;

	it.node()->get()->InitHash();
	cout << "edge: " << endl << it.node()->get()->edgeHash << endl;
	cout << "edge address: " << &(it.node()->get()->edge) << endl;
	cout << "face: " << endl << it.node()->get()->faceHash << endl;

	cout << "Face Edge: " << endl;

	for (int i = 0; i < it.node()->get()->faceEdgeHash.size(); i++)
	{
		cout << it.node()->get()->faceEdgeHash[i] << endl;
	}

	cout << endl;
}



#endif
