#clear(VTKDIR)
#equals(QMAKE_HOST.os, Windows) {
#    VTKDIR = "E:/software/vtk8/vtk8"
#    #vtkstatic: VTKDIR = $${EXTDIR}/VTK
#}

#INCLUDEPATH += $${VTKDIR}/include

#CONFIG(debug, debug|release) {
#    LIBS += -L$${VTKDIR}/lib/Debug -L$${VTKDIR}/b64/lib/Debug
#} else {
#    LIBS += -L$${VTKDIR}/lib/Release -L$${VTKDIR}/b64/lib/Release
#}
#LIBS += -L$${VTKDIR}/lib

#DEFINES += VTK_USE_SYSTEM_FREETYPE VTK_USE_SYSTEM_SQLITE VTK_USE_SYSTEM_ZLIB VTK_USE_SYSTEM_EXPAT VTK_USE_SYSTEM_GLEW
#vtkstatic: DEFINES += VTKCOMMONCORE_STATIC_DEFINE VTKCOMMONTRANSFORMS_STATIC_DEFINE VTKCOMMONCOLOR_STATIC_DEFINE VTKCOMMONCOMPUTATIONALGEOMETRY_STATIC_DEFINE \
#    VTKCOMMONDATAMODEL_STATIC_DEFINE VTKCOMMONEXECUTIONMODEL_STATIC_DEFINE VTKCOMMONMATH_STATIC_DEFINE VTKCOMMONMISC_STATIC_DEFINE VTKCOMMONSYSTEM_STATIC_DEFINE \
#    VTKFILTERSCORE_STATIC_DEFINE VTKFILTERSSOURCES_STATIC_DEFINE VTKFILTERSGENERAL_STATIC_DEFINE VTKFILTERSEXTRACTION_STATIC_DEFINE \
#    VTKFILTERSMODELING_STATIC_DEFINE VTKFILTERSGEOMETRY_STATIC_DEFINE VTKFILTERSHYBRID_STATIC_DEFINE VTKFILTERSPOINTS_STATIC_DEFINE \
#    VTKIMAGINGCORE_STATIC_DEFINE VTKIMAGINGHYBRID_STATIC_DEFINE VTKIMAGINGGENERAL_STATIC_DEFINE VTKIMAGINGSTENCIL_STATIC_DEFINE \
#    VTKIOCORE_STATIC_DEFINE VTKIOLEGACY_STATIC_DEFINE VTKIOPLY_STATIC_DEFINE VTKIOIMAGE_STATIC_DEFINE VTKIOGEOMETRY_STATIC_DEFINE VTKIOPARALLEL_STATIC_DEFINE \
#    VTKIOXML_STATIC_DEFINE \
#    VTKRENDERINGCORE_STATIC_DEFINE VTKRENDERINGOPENGL2_STATIC_DEFINE VTKRENDERINGVOLUMEOPENGL2_STATIC_DEFINE VTKRENDERINGFREETYPE_STATIC_DEFINE \
#    VTKRENDERINGVOLUME_STATIC_DEFINE VTKRENDERINGANNOTATION_STATIC_DEFINE VTKRENDERINGCONTEXT2D_STATIC_DEFINE \
#    VTKINTERACTIONSTYLE_STATIC_DEFINE VTKINTERACTIONIMAGE_STATIC_DEFINE VTKINTERACTIONWIDGETS_STATIC_DEFINE VTKGUISUPPORTQT_STATIC_DEFINE \
#    VTKCHARTSCOREE_STATIC_DEFINE VTKVIEWSCONTEXT2D_STATIC_DEFINE \
#    VTKPARALLELCORE_STATIC_DEFINE

#DEPENDS += freetype glew zlib
##VTKDEPENDS += vtkfreetype vtkglew vtkzlib
#VTKDEPENDS += vtkexpat vtklz4 vtklzma vtkpng vtkjpeg
#VTKDEPENDS += vtksys vtkCommonCore vtkCommonTransforms vtkCommonComputationalGeometry vtkCommonDataModel vtkCommonExecutionModel vtkCommonMath \
#    vtkCommonMisc vtkCommonSystem vtkCommonColor \
#    vtkFiltersCore vtkFiltersSources vtkFiltersGeneral vtkFiltersExtraction vtkFiltersModeling vtkFiltersGeometry vtkFiltersHybrid vtkFiltersPoints \
#    vtkImagingCore vtkImagingMath vtkImagingHybrid vtkImagingGeneral vtkImagingStencil \
#    vtkIOCore vtkIOLegacy vtkIOPLY vtkIOGeometry vtkIOParallel vtkIOImage vtkmetaio vtkDICOMParser \
##    vtkIOXML vtkIOXMLParser \
#    vtkRenderingCore vtkRenderingOpenGL2 vtkRenderingVolumeOpenGL2 vtkRenderingFreeType vtkRenderingVolume vtkRenderingAnnotation vtkRenderingContext2D \
#    vtkInteractionStyle vtkInteractionImage vtkInteractionWidgets vtkGUISupportQt \
##    vtkChartsCore vtkViewsContext2D \
#    vtkParallelCore \

#for(i, VTKDEPENDS): LIBS += -l$${i}-8.2

#win32: LIBS += -lOpenGL32 -lDbgHelp
