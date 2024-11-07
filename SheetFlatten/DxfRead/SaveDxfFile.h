#ifndef _DrawDxfResult_hxx
#define _DrawDxfResult_hxx

//////////////////////////////////////////////////////////////////////////////////////
// put your codes here
//标注块的写入
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>


#include "../DxfCore/dl_dxf.h"
#include "DxfCoreData.hxx"
using namespace dxf::core;

class SaveDxfFile
{
public:

    //// 保展开图结果，将上面容器中的数据进行分层保存DXF
    void SaveDxf( const std::vector<std::tuple<point_t, point_t, std::string>>& vec_AllCaseLine,
					const std::vector<std::tuple<point_t, double, std::string>>& vec_AllCaseCircle,
					const std::vector< std::tuple< point_t, double, double, double, std::string > > vec_AllCaseArc,

					const std::string DxfFileName 
                );
};
SaveDxfFile class_SaveDxfFile;





//// 将上面容器中的数据进行分层保存DXF
void SaveDxfFile::SaveDxf( const std::vector<std::tuple<point_t, point_t, std::string>>& vec_AllCaseLine,
                            const std::vector<std::tuple<point_t, double, std::string>>& vec_AllCaseCircle,
                            const std::vector< std::tuple< point_t, double, double, double, std::string > > vec_AllCaseArc,
                            const std::string DxfFileName 
                        )
{
    const char* c_file_name = DxfFileName.c_str();

    DL_Dxf dxf;
    DL_WriterA* dw = dxf.out(c_file_name, DL_Codes::AC1015); 

    // section header:
    dxf.writeHeader(*dw);
    dw->sectionEnd();

    // section tables:
    dw->sectionTables();

    // VPORT:
    dxf.writeVPort(*dw);

    // LTYPE:
    dw->tableLinetypes(3);
    dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
    dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
    dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
    dw->tableEnd();

    // LAYER:
    std::vector < std::string > Layer_Name = { "0", "外轮廓线"};
    dw->tableLayers(Layer_Name.size());
    for (size_t OrderLayerName = 0; OrderLayerName < Layer_Name.size(); ++OrderLayerName)
    {
        const auto& itemLayerName = Layer_Name[OrderLayerName];
        dxf.writeLayer(
                        *dw,
                        DL_LayerData(itemLayerName, 0),
                        DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS")
                    );
    }
    dw->tableEnd();

    // STYLE:
    dw->tableStyle(1);
    DL_StyleData style("Standard", 0, 0.0, 1.0, 0.0, 0, 2.5, "txt", "");
    style.bold = false;
    style.italic = false;
    dxf.writeStyle(*dw, style);
    dw->tableEnd();

    // VIEW:
    dxf.writeView(*dw);

    // UCS:
    dxf.writeUcs(*dw);

    // APPID:
    dw->tableAppid(1);
    dxf.writeAppid(*dw, "ACAD");
    dw->tableEnd();

    // DIMSTYLE:
    dxf.writeDimStyle(*dw, 2.5, 1, 1, 1, 20);

    // BLOCK_RECORD:
    dxf.writeBlockRecord(*dw);
    dw->tableEnd();

    dw->sectionEnd();

    // BLOCK:
    dw->sectionBlocks();
    dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Model_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space");
    dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
    dxf.writeEndBlock(*dw, "*Paper_Space0");
    dw->sectionEnd();

    // ENTITIES:
    dw->sectionEntities();
    
    //// 展开图中的线条
    if( vec_AllCaseLine.size() != 0 )
        for (size_t i = 0; i <= vec_AllCaseLine.size() - 1; ++i)
        {
            const auto& item = vec_AllCaseLine[i];
			dxf.writeLine(  *dw, 
							DL_LineData(std::get<0>(item).x, std::get<0>(item).y, 0, std::get<1>(item).x, std::get<1>(item).y, 0),
							DL_Attributes("外轮廓线", 10, -1, -1, "BYLAYER")
						);            
        }


    //// 保存展开图中的圆和异形角码孔
    if( vec_AllCaseCircle.size() != 0 )
        for (size_t i = 0; i <= vec_AllCaseCircle.size() - 1; ++i)
        {
            const auto& item = vec_AllCaseCircle[i];

            dxf.writeCircle(*dw, 
                    DL_CircleData(std::get<0>(item).x, std::get<0>(item).y, 0, std::get<1>(item)),
                    DL_Attributes("外轮廓线",  1, -1, -1.0, "BYLAYER")
            );
        }


     // 写多段线的 std::vector<std::vector<std::pair<dxf::core::point_t, dxf::core::point_t>>> AllCasePlane
   /*  for(size_t Order0 = 0; Order0 <= AllCasePlane.size() - 1; ++Order0)
     {
         const auto & Element0_0 = AllCasePlane[Order0];

         dxf.writePolyline(  *dw, 
                             DL_PolylineData( Element0_0.size() + 1, 0, 0, 0, 0 ),
                             DL_Attributes("外轮廓线", 10, -1, -1, "BYLAYER")
                     );
         for( size_t Order1 = 0; Order1 <= Element0_0.size() - 1; ++Order1 )
         {
             const auto & Element1_0 = Element0_0[Order1];

             if( Order1 == 0 )
             {
                 dxf.writeVertex( *dw, DL_VertexData(Element1_0.first.x, Element1_0.first.y, 0) );
                 dxf.writeVertex( *dw, DL_VertexData(Element1_0.second.x, Element1_0.second.y, 0) );
             }
             else
             {
                 dxf.writeVertex( *dw, DL_VertexData(Element1_0.second.x, Element1_0.second.y, 0) );
             }

         }
     }*/


    //// 写入圆弧数据
    if( vec_AllCaseArc.size() != 0 )
        for( int Order0 = 0; Order0 <= vec_AllCaseArc.size() - 1; ++Order0 )
        {
			dxf.writeArc(*dw, 
					DL_ArcData( std::get<0>(vec_AllCaseArc[Order0]).x, std::get<0>(vec_AllCaseArc[Order0]).y, std::get<0>(vec_AllCaseArc[Order0]).z, 
								std::get<1>(vec_AllCaseArc[Order0]), std::get<2>(vec_AllCaseArc[Order0]), std::get<3>(vec_AllCaseArc[Order0]) ),
					DL_Attributes( "外轮廓线",  10, -1, -1.0, "BYLAYER" )
			);
        }



    // end section ENTITIES:
    dw->sectionEnd();
    dxf.writeObjects(*dw, "MY_OBJECTS");
    dxf.writeObjectsEnd(*dw);

    dw->dxfEOF();
    dw->close();
    delete dw;
}




#endif
