#ifndef SHEETFLATTENCORE_H
#define SHEETFLATTENCORE_H

#include <GeomAbs_SurfaceType.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <GeomConvert_SurfToAnaSurf.hxx>
#include <GeomConvert_CurveToAnaCurve.hxx>
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

#include "ModelIO/ModelIO.h"
#include "tcl_5.0.6/tree.h"
#include "SheetFlattenBase.h"

#include <iostream>
#include <vector>

using namespace std;
using namespace tcl;



void BuildDocStd(const TopoDS_Shape& shape)
{
	Handle(TDocStd_Application) app = new TDocStd_Application;
	Handle(TDocStd_Document) doc;
	app->NewDocument("BinXCAF", doc);

	TDF_Label mainLab = doc->Main();
	Handle(XCAFDoc_ShapeTool)ST = XCAFDoc_DocumentTool::ShapeTool(doc->Main());

	ST->AddShape(shape);

	ModelIO modelio;
	modelio.Write("F:\\test.stl", doc, GeomSTL);
	//modelio.Write("F:\\test.stp", doc, GeomSTP);

	app->Close(doc);
}

struct FlattenFaceNode
{
	TopoDS_Edge edge;
	FlattenFace flattenFace;
	int startNodeId;
	int endNodeId;
	Node startNode;
	Node endNode;

	int edgeHash;
	vector<int> faceEdgeHash;

	void InitHash()
	{
		for (TopExp_Explorer edgeExp(flattenFace.face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

			faceEdgeHash.push_back(hash<TopoDS_Edge>{}(objEdge));

		}

		edgeHash = hash<TopoDS_Edge>{}(edge);
	}

	bool isNull()
	{
		if (flattenFace.face.IsNull())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool isRoot()
	{
		if (edge.IsNull())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void ParseNode()
	{
		TopLoc_Location location;
		opencascade::handle<Poly_Triangulation> triFace = BRep_Tool::Triangulation(flattenFace.face, location);

		TopLoc_Location locationEdge;

		for (TopExp_Explorer edgeExp(flattenFace.face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

			cout << hash<TopoDS_Edge>{}(objEdge) << endl;

		}

		opencascade::handle<Poly_PolygonOnTriangulation> triEdge = BRep_Tool::PolygonOnTriangulation(edge, triFace, locationEdge);

		const TColStd_Array1OfInteger& nds = triEdge->Nodes();

		startNodeId = nds[1];
		startNode = flattenFace.GetNode(nds[1]);

		if (nds[1] == nds[nds.Size()])
		{
			endNodeId = nds[nds.Size() - 1];
			endNode = flattenFace.GetNode(nds[nds.Size() - 1]);
		}
		else
		{
			endNodeId = nds[nds.Size()];
			endNode = flattenFace.GetNode(nds[nds.Size()]);
		}
	}

	Node StartNode()
	{
		return startNode;
	}

	Node EndNode()
	{
		return endNode;
	}

	gp_Pnt2d StartNode2D()
	{
		return flattenFace.GetJigsawNode(startNodeId);
	}

	gp_Pnt2d EndNode2D()
	{
		return flattenFace.GetJigsawNode(endNodeId);
	}

	friend bool operator < (const FlattenFaceNode& lhs, const FlattenFaceNode& rhs)
	{
		return lhs.flattenFace.face < rhs.flattenFace.face;
	}

	friend bool operator == (const FlattenFaceNode& lhs, const FlattenFaceNode& rhs)
	{
		return lhs.flattenFace.face == rhs.flattenFace.face;
	}
};


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
		//aMeshParams.MeshAlgo = IMeshTools_MeshAlgoType_Delabella;

		//CurveDeflection = 0.001;
		//AngularDeflection = 1 / M_PI_4;

		faceid = 0;
	}
public:
	bool Read(const string& filePath);
	int Perform();

private:

	void BuildEdgeAdjFacesMap(const TopoDS_Shape &objShape);
	bool BuildFlattenTree(const TopoDS_Shape& objShape);
	bool isFacePlane(const TopoDS_Face& face);
	void BuildFlattenTreeNode(tree<FlattenFaceNode>::iterator it);
	void Jigsaw(tree<FlattenFaceNode>::iterator rootIt);

	void DumpTree(tree<FlattenFaceNode>::iterator it);


private:
	Handle(TDocStd_Document) doc;
	TopoDS_Compound aCompound;
	BRep_Builder aBuilder;
	IMeshTools_Parameters aMeshParams;

	FlattenFaceNode rootNode;		// 这里定义这个变量是为了对应指定基准面展开时，交互定义

	tree< FlattenFaceNode> shapeTree;

	vector<FlattenFace> aFlattenFaces;

	map<TopoDS_Edge, set<TopoDS_Face> > mapEdgeAdjFaces;
	set<TopoDS_Face> finishedFaces;

	int faceid;

};

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

		//cout << mapEdgeAdjFaces.size() << endl;

		//for (map<TopoDS_Edge, set<TopoDS_Face>>::iterator it = mapEdgeAdjFaces.begin(); it != mapEdgeAdjFaces.end(); it++)
		//{
		//	cout << it->second.size() << endl;
		//}

		if (!BuildFlattenTree(objShape))
		{
			return 1;	// 没有root face
		}		
	}

	//BuildDocStd(aCompound);

	return 0;
}

inline void SheetFlattenCore::BuildEdgeAdjFacesMap(const TopoDS_Shape &objShape)
{
	for (TopExp_Explorer faceExp(objShape, TopAbs_FACE); faceExp.More(); faceExp.Next())
	{
		const TopoDS_Face& objFace = TopoDS::Face(faceExp.Current());

		for (TopExp_Explorer edgeExp(objFace, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

			if (mapEdgeAdjFaces.find(objEdge) != mapEdgeAdjFaces.end())
			{
				mapEdgeAdjFaces[objEdge].insert(objFace);
			}
			else
			{
				set<TopoDS_Face> aFaces;
				aFaces.insert(objFace);
				mapEdgeAdjFaces.insert(make_pair(objEdge, aFaces));
			}
		}
	}
}

inline bool SheetFlattenCore::BuildFlattenTree(const TopoDS_Shape& objShape)
{
	// 如果没有指定root face，则找出面积最大的平面
	if (rootNode.isNull())
	{
		// 找到面积最大的平面作为root face
		double area = -1e6;

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
				FlattenFace flattenFace(objFace);
				rootNode.flattenFace = flattenFace;
				area = fabs(props.Mass());
			}
		}
	}

	// 如果没有平面
	if (rootNode.isNull())
	{
		return false;
	}
	else
	{
		rootNode.flattenFace.OneStepFlattenFace();
	}

	tree<FlattenFaceNode>::iterator it = shapeTree.insert(rootNode);
	finishedFaces.insert(rootNode.flattenFace.face);

	vector<tree<FlattenFaceNode>::iterator> firstRound;

	for (TopExp_Explorer edgeExp(rootNode.flattenFace.face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
	{
		const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

		for (set<TopoDS_Face>::iterator faceIt = mapEdgeAdjFaces[objEdge].begin(); faceIt != mapEdgeAdjFaces[objEdge].end(); faceIt++)
		{
			// 一个边只包含两个相邻面所以，只需要判断是否是已展开的面即可，不用对比是不是与faceIt相同
			if (finishedFaces.find(*faceIt) != finishedFaces.end())
			{
				continue;
			}

			FlattenFaceNode _node;
			_node.flattenFace.Init(*faceIt);
			_node.edge = objEdge;			

			_node.flattenFace.OneStepFlattenFace();
			_node.flattenFace.Transform2ZPlane();

			finishedFaces.insert(*faceIt);
			tree<FlattenFaceNode>::iterator child_it = it.node()->insert(_node);
			firstRound.push_back(child_it);
		}
	}

	for (int i = 0; i < firstRound.size(); i++)
	{
		BuildFlattenTreeNode(firstRound[i]);
	}	

	DumpTree(it);

	it.node()->get()->InitHash();
	cout << it.node()->get()->edgeHash << endl;

	for (int i = 0; i < it.node()->get()->faceEdgeHash.size(); i++)
	{
		cout << it.node()->get()->faceEdgeHash[i] << endl;
	}

	cout << endl;
	
	Jigsaw(it);
	return true;
}

inline void SheetFlattenCore::DumpTree(tree<FlattenFaceNode>::iterator it)
{
	for (typename tree<FlattenFaceNode>::iterator iter = it.node()->begin(); iter != it.node()->end(); iter++)
	{
		iter.node()->get()->InitHash();
		cout << iter.node()->get()->edgeHash << endl;

		for (int i = 0; i < iter.node()->get()->faceEdgeHash.size(); i++)
		{
			cout << iter.node()->get()->faceEdgeHash[i] << endl;
		}

		cout << endl;

		DumpTree(iter);
	}
}

inline void SheetFlattenCore::BuildFlattenTreeNode(tree<FlattenFaceNode>::iterator it)
{
	vector<tree<FlattenFaceNode>::iterator> firstRound;

	for (TopExp_Explorer edgeExp(it.node()->get()->flattenFace.face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
	{
		const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

		for (set<TopoDS_Face>::iterator faceIt = mapEdgeAdjFaces[objEdge].begin(); faceIt != mapEdgeAdjFaces[objEdge].end(); faceIt++)
		{
			// 一个边只包含两个相邻面所以，只需要判断是否是已展开的面即可，不许对比是不是与faceIt相同
			if (finishedFaces.find(*faceIt) != finishedFaces.end())
			{
				continue;
			}

			FlattenFaceNode _node;
			_node.flattenFace.Init(*faceIt);
			_node.edge = objEdge;

			_node.flattenFace.OneStepFlattenFace();
			_node.flattenFace.Transform2ZPlane();

			finishedFaces.insert(*faceIt);
			tree<FlattenFaceNode>::iterator child_it = it.node()->insert(_node);
			firstRound.push_back(child_it);
		}
	}

	for (int i = 0; i < firstRound.size(); i++)
	{
		BuildFlattenTreeNode(firstRound[i]);
	}
	//const TopoDS_Edge& edge = it->edge;

	//for (set<TopoDS_Face>::iterator faceIt = mapEdgeAdjFaces[edge].begin(); faceIt != mapEdgeAdjFaces[edge].end(); faceIt++)
	//{
	//	// 一个边只包含两个相邻面所以，只需要判断是否是已展开的面即可，不许对比是不是与faceIt相同
	//	if (finishedFaces.find(*faceIt) != finishedFaces.end())
	//	{
	//		continue;
	//	}

	//	FlattenFaceNode _node;
	//	_node.flattenFace.Init(*faceIt);
	//	_node.edge = edge;

	//	_node.flattenFace.OneStepFlattenFace();
	//	//_node.flattenFace.Transform2ZPlane();

	//	finishedFaces.insert(*faceIt);
	//	tree<FlattenFaceNode>::iterator child_it = it.node()->insert(_node);
	//	BuildFlattenTreeNode(child_it);		
	//}
}
//inline void SheetFlattenCore::Jigsaw(tree<FlattenFaceNode>* faceNode)
inline void SheetFlattenCore::Jigsaw(tree<FlattenFaceNode>::iterator rootIt)
{	
	// 如果是根节点，则先展平根节点；
	if (rootIt.node()->get()->isRoot())
	{
		rootIt.node()->get()->flattenFace.Transform2ZPlane();
	}

	FlattenFaceNode *fatherFlattenNode = rootIt.node()->get();

	// 遍历子节点
	for (typename tree<FlattenFaceNode>::iterator it = rootIt.node()->begin(); it != rootIt.node()->end(); it++)
	{
		fatherFlattenNode->ParseNode();
		it.node()->get()->flattenFace.Transform2ZPlane(it.node()->get()->edge, fatherFlattenNode->StartNode(), fatherFlattenNode->EndNode());
		it.node()->get()->flattenFace.Jigsaw(fatherFlattenNode->StartNode2D(), fatherFlattenNode->EndNode2D());

		string nasFile = "F:\\Face_" + to_string(faceid++) + ".nas";
		it.node()->get()->flattenFace.DumpNasEx(nasFile, 3);

		Jigsaw(it);		
	}
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




#endif
