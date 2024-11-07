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

	//ModelIO modelio;
	//modelio.Write("F:\\test.stl", doc, GeomSTL);
	//modelio.Write("F:\\test.stp", doc, GeomSTP);

	app->Close(doc);
}

struct FlattenFaceNode
{
	TopoDS_Edge edge;
	FlattenFace *flattenFace;
	int startNodeId;
	int endNodeId;
	Node startNode;
	Node endNode;
	FaceOrientation faceOri;

	int id;

	size_t edgeHash;
	size_t faceHash;
	vector<size_t> faceEdgeHash;

	void InitHash()
	{
		faceEdgeHash.clear();
		for (TopExp_Explorer edgeExp(flattenFace->face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

			faceEdgeHash.push_back(hash<TopoDS_Edge>{}(objEdge));

		}

		edgeHash = hash<TopoDS_Edge>{}(edge);
		faceHash = hash<TopoDS_Face>{}(flattenFace->face);
	}

	bool isNull()
	{
		if (flattenFace == nullptr)
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

	void ParseNode(const TopoDS_Edge& objEdge)
	{
		TopLoc_Location location;
		opencascade::handle<Poly_Triangulation> triFace = BRep_Tool::Triangulation(flattenFace->face, location);
		opencascade::handle<Poly_PolygonOnTriangulation> triEdge;

		for (TopExp_Explorer edgeExp(flattenFace->face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
		{
			const TopoDS_Edge& _edge = TopoDS::Edge(edgeExp.Current());

			if (hash<TopoDS_Edge>{}(objEdge) == hash<TopoDS_Edge>{}(_edge))
			{
				triEdge = BRep_Tool::PolygonOnTriangulation(_edge, triFace, location);
				break;
			}
		}

		const TColStd_Array1OfInteger& nds = triEdge->Nodes();

		startNodeId = nds[1];
		startNode = flattenFace->GetNode(nds[1]);

		if (nds[1] == nds[nds.Size()])
		{
			endNodeId = nds[nds.Size() - 1];
			endNode = flattenFace->GetNode(nds[nds.Size() - 1]);
		}
		else
		{
			endNodeId = nds[nds.Size()];
			endNode = flattenFace->GetNode(nds[nds.Size()]);
		}		

		faceOri = flattenFace->GetOrientation(startNodeId, endNodeId);
	}

	FaceOrientation GetOrientation()
	{
		return faceOri;
	}

	Node StartNode()
	{
		return flattenFace->GetNode(startNodeId);
	}

	Node EndNode()
	{
		return flattenFace->GetNode(endNodeId);
	}

	Node StartNodeFlat()
	{
		return flattenFace->GetFlatNode(startNodeId);
	}

	Node EndNodeFlat()
	{
		return flattenFace->GetFlatNode(endNodeId);
	}

	gp_Pnt2d StartNode2D()
	{
		return flattenFace->GetJigsawNode(startNodeId);
	}

	gp_Pnt2d EndNode2D()
	{
		return flattenFace->GetJigsawNode(endNodeId);
	}

	friend bool operator < (const FlattenFaceNode& lhs, const FlattenFaceNode& rhs)
	{
		return lhs.flattenFace->face < rhs.flattenFace->face;
	}

	friend bool operator == (const FlattenFaceNode& lhs, const FlattenFaceNode& rhs)
	{
		return lhs.flattenFace->face == rhs.flattenFace->face;
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
		cout << "aMeshParams.MinSize: " << aMeshParams.MinSize << endl;
		aMeshParams.InternalVerticesMode = Standard_True;
		aMeshParams.ControlSurfaceDeflection = Standard_True;
		//aMeshParams.MeshAlgo = IMeshTools_MeshAlgoType_Delabella;

		//CurveDeflection = 0.001;
		//AngularDeflection = 1 / M_PI_4;

		faceid = 0;
		refId = 0;
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

	void DumpTree(tree<FlattenFaceNode>::pre_order_iterator it);
	void DumpTree(tree<FlattenFaceNode>::level_order_iterator it);
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
	int refId;

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
	cout << "BuildFlattenTree" << endl;
	// 如果没有指定root face，则找出面积最大的平面
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

		rootNode.flattenFace->Init(maxAreaFace);

	}

	// 如果没有平面
	if (rootNode.isNull())
	{
		return false;
	}
	else
	{
		rootNode.flattenFace->OneStepFlattenFace();

		rootNode.flattenFace->DumpNasEx("F:\\FaceInit_" + to_string(faceid) + ".nas", 0);

		string nasFile = "F:\\FaceOnestep_" + to_string(faceid++) + ".nas";
		
		rootNode.flattenFace->DumpNasEx(nasFile, 1);
	}

	rootNode.id = ++refId;

	tree<FlattenFaceNode>::iterator it = shapeTree.insert(rootNode);	
	finishedFaces.insert(rootNode.flattenFace->face);
	cout << "BuildFlattenTreeNode" << endl;
	DumpTree(it);
	BuildFlattenTreeNode(it);
	cout << "finishedFaces hash: " << endl;
	for (set<TopoDS_Face>::iterator iter = finishedFaces.begin(); iter != finishedFaces.end(); iter++)
	{
		cout << hash<TopoDS_Face>{}(*iter) << endl;
	}

	for (typename tree<FlattenFaceNode>::pre_order_iterator pit = shapeTree.pre_order_begin(); pit != shapeTree.pre_order_end(); pit++)
	{
		cout << pit.node()->get()->id << endl;
	}

	Jigsaw(it);
	return true;
}


inline void SheetFlattenCore::BuildFlattenTreeNode(tree<FlattenFaceNode>::iterator it)
{
	vector<tree<FlattenFaceNode>::iterator> firstRound;

	cout << hash<TopoDS_Face>{}(it.node()->get()->flattenFace->face) << endl;

	for (TopExp_Explorer edgeExp(it.node()->get()->flattenFace->face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next())
	{
		const TopoDS_Edge& objEdge = TopoDS::Edge(edgeExp.Current());

		for (set<TopoDS_Face>::iterator faceIt = mapEdgeAdjFaces[objEdge].begin(); faceIt != mapEdgeAdjFaces[objEdge].end(); faceIt++)
		{
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
			_node.flattenFace->Init(*faceIt);
			_node.edge = objEdge;

			_node.flattenFace->DumpNasEx("F:\\FaceInit_" + to_string(faceid) + ".nas", 0);
			_node.flattenFace->OneStepFlattenFace();
			string nasFile = "F:\\FaceOnestep_" + to_string(faceid++) + ".nas";
			_node.flattenFace->DumpNasEx(nasFile, 1);
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
	cout << "DumpTree fatherIt:" << endl;
	DumpTree(fatherIt);

	faceid = 0;
	

	// 遍历子节点
	typename tree<FlattenFaceNode>::level_order_iterator lit = fatherIt.node()->level_order_begin();
	lit.node()->get()->flattenFace->Transform2ZPlane();
	lit.node()->get()->flattenFace->DumpNasEx("F:\\Face2ZPlane_" + to_string(faceid) + ".nas", 2);
	lit.node()->get()->flattenFace->DumpNasEx("F:\\FaceJigsaw_" + to_string(faceid++) + ".nas", 3);
	lit++;
	for ( ;lit != fatherIt.node()->level_order_end(); lit++)
	{
		cout << "DumpTree child It:" << endl;
		DumpTree(lit);
		FlattenFaceNode* fatherFlattenNode = lit.node()->parent()->get();
		fatherFlattenNode->ParseNode(lit.node()->get()->edge);
		lit.node()->get()->flattenFace->Transform2ZPlane(lit.node()->get()->edge, fatherFlattenNode->StartNode(), fatherFlattenNode->EndNode(), fatherFlattenNode->GetOrientation());

		lit.node()->get()->flattenFace->DumpNasEx("F:\\Face2ZPlane_" + to_string(faceid) + ".nas", 2);

		lit.node()->get()->flattenFace->Jigsaw(fatherFlattenNode->StartNode2D(), fatherFlattenNode->EndNode2D());

		string nasFile = "F:\\FaceJigsaw_" + to_string(faceid++) + ".nas";

		lit.node()->get()->flattenFace->DumpNasEx(nasFile, 3);
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
