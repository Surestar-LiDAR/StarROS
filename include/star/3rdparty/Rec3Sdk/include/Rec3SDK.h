/**
 * This SDK allows you to read and write structured or unstructured point clouds in Reconstructor 3 file format.
 * 
 * Only one C-style header file. All symbols have external C linkage, to enable compatibility across many C++ compilers.
 * To avoid name clashes, all symbols names begin with "Rec_".
 *
 * To submit questions or bugs, please contact support@gexcel.it.
 */

#ifndef RECONSTRUCTOR3_SDK_GEXCEL_20150923_NF4738QDHFUIFH384FGY7ERBFUDKADS3W478RSMKABDWEHEYFG3Y48MDF0IOWAMBMF4MRBODUDFAOMW8EZMXBHFHBU8QSBFME38FB8UF7Z3
#define RECONSTRUCTOR3_SDK_GEXCEL_20150923_NF4738QDHFUIFH384FGY7ERBFUDKADS3W478RSMKABDWEHEYFG3Y48MDF0IOWAMBMF4MRBODUDFAOMW8EZMXBHFHBU8QSBFME38FB8UF7Z3

#ifdef REC3SDK_LIB
#define REC3SDK_DECL __declspec(dllexport)
#else
#define REC3SDK_DECL __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	struct Rec_PointCloud;
	struct Rec_ColorLayer;

	typedef const Rec_PointCloud* Rec_PointCloudPtr;
	typedef const Rec_ColorLayer* Rec_ColorLayerPtr;

	/**
	 * The SDK is composed mainly of C-style functions. These functions never throw exceptions. Instead, after calling any function, you can call Rec_lastErrorCode()
	 * to check if everything is ok ( e.g. if(Rec_lastErrorCode() != REC_OK) { handle errors...} )
	 */
	enum Rec_ErrorCode
	{
		REC_OK = 0,
		REC_UNKNOWN_ERROR = 1,
		REC_FILE_DOES_NOT_EXIST = 2,
		REC_XML_PARSING_ERROR = 3,
		REC_CANNOT_WRITE_FILE = 4,
		REC_CALL_INVALID_FOR_CLOUD_NOT_STRUCTURED = 5,
		REC_INVALID_ARGUMENTS_IN_CALL = 6,
		REC_ERROR_IN_READING_BINARY_FILE = 7,
		REC_DATA_COMPRESSION_ERROR = 8,
		REC_INVALID_POINT_CLOUD_POINTER = 9,
		REC_UNRECOGNIZED_FILE_FORMAT = 10,
		REC_POINT_INDEX_OUT_OF_RANGE = 11,
		REC_COLOR_LAYER_INDEX_OUT_OF_RANGE = 12,
		REC_INVALID_COLOR_LAYER_POINTER = 13,
		REC_INVALID_COLOR_DATATYPE_STRING = 14,
		REC_COLOR_LAYER_DYNAMIC_TYPECHECK_FAILED = 15,
		REC_CANNOT_CREATE_COLOR_LAYER_NAME_ALREADY_IN_USE = 16,
		REC_NOT_ENOUGH_RAM_MEMORY = 17
	};

	/**In Reconstructor, these are the flags that can be assigned to each point of a point cloud.*/
	union Rec_PointFlags
	{
		unsigned int status;
		struct
		{
			bool Invalid : 1;                   // Point is not defined at all
			bool Deleted : 1;                   // Deleted by user
			bool ValueOutOfBounds : 1;          // Discarded by Reconstructor's range and reflectance filter
			bool MixedPoint : 1;                // Discarded, point is too steep from scanning direction (relative depth discontinuity)
			bool OrientationDiscontinuity : 1;  // Point is on a normal discontinuity edge (e.g. a corner)
			bool DepthDiscontinuity : 1;        // Point is on an absolute depth discontinuity edge
			bool Overlapping : 1;               // Redundant point for integration
			bool UserMask : 1;                  // User defined bit mask
			bool AuxMask : 1;                   // Auxiliary bit, used by some algorithm to flag it
			bool Outlier : 1;                   // Discarded because too far away from its neighbours
		};
	};

	/**A point is invalid iff its Invalid flag is true.*/
	REC3SDK_DECL	extern const unsigned int REC_FLAGS_INVALID_POINT;

	/**A point is drawable iff none of the following flags is true: Invalid, Deleted, ValueOutOfBounds, MixedPoint, Overlapping, UserMask, Outlier
	 * To quickly check if a point is drawable or not: bool drawable = (flags.status & REC_FLAGS_POINT_NOT_DRAWABLE) == 0;
	 */
	REC3SDK_DECL	extern const unsigned int REC_FLAGS_POINT_NOT_DRAWABLE;

	/**A point is meshable iff is drawable AND its flags DepthDiscontinuity and AuxMask are off (you can draw depth discontinuities but you cannot mesh them).
	 * To quickly check if a point is meshable or not: bool meshable = (flags.status & REC_FLAGS_POINT_NOT_MESHABLE) == 0;
	 */
	REC3SDK_DECL	extern const unsigned int REC_FLAGS_POINT_NOT_MESHABLE;

	/**
	 * Gets the error code returned by last call to the SDK.
	 */
	REC3SDK_DECL	Rec_ErrorCode		Rec_lastErrorCode();

	/**
	 * Gets the full error message returned by last call to the SDK, for example "error while parsing xml file c:/.../thecloud.rgp, at line 5, column28".
	 */
	REC3SDK_DECL	const wchar_t*		Rec_lastErrorMessage();

	/**
	 * Reads the point cloud file and returns a new Rec_PointCloudPtr to read and write data to the cloud.
	 * "fileName" must contain the absolute path to an existing point cloud file.
	 * Memory ownership: the SDK deallocates the returned pointer, therfore do NOT delete it otherwise the program crashes.
	 */
	REC3SDK_DECL	Rec_PointCloudPtr	Rec_openFile(const wchar_t* fileName);

	/**READING POINT CLOUDS' DATA AND PROPERTIES*/

	/**Returns whether the given cloud is structured. A cloud is structured iff each point has a unique (row, column) index.*/
	REC3SDK_DECL	bool				Rec_PointCloud_isStructured(Rec_PointCloudPtr theCloud);

	/**Returns the total number of points contained in the given cloud, including invalid or deleted points.*/
	REC3SDK_DECL	unsigned int		Rec_PointCloud_getNumPoints(Rec_PointCloudPtr theCloud);

	/**Returns the number of valid points. A points is valid iff is drawable, or iff its flags satisfy (flags.status & REC_FLAGS_NOT_DRAWABLE) == 0*/
	REC3SDK_DECL	unsigned int		Rec_PointCloud_getNumValidPoints(Rec_PointCloudPtr theCloud);

	/**Returns how many color layers are stored in the point cloud. In Reconstructor 3, a point cloud can have an unlimited number of color layers,
	 * containing information about the points' reflectance, color, normals, confidence, altitudes with respect to a reference plane, distance to a reference model, etc.
	 */
	REC3SDK_DECL	int					Rec_PointCloud_getNumColorLayers(Rec_PointCloudPtr theCloud);

	/** Returns a pointer to the i-th color layer of the given cloud. The SDK deallocates the returned pointer, therefore the client program should not delete it.	 */
	REC3SDK_DECL	Rec_ColorLayerPtr	Rec_PointCloud_getColorLayer(Rec_PointCloudPtr theCloud, int colorLayerIdx);

	/** Returns a pointer to the color layer with the given name, or null if such color layer does not exist. */
	REC3SDK_DECL	Rec_ColorLayerPtr   Rec_PointCloud_findColorLayerByName(Rec_PointCloudPtr theCloud, const char* colorName);	

	/** Returns the absolute filepath of the given point cloud. The SDK owns the memory pointed by the returned pointer. */
	REC3SDK_DECL	const wchar_t*		Rec_PointCloud_getFilepath(Rec_PointCloudPtr theCloud);

	/**Functions to get width and height of a structured point cloud.*/
	REC3SDK_DECL	unsigned int		Rec_PointCloud_getWidth(Rec_PointCloudPtr theCloud);

	REC3SDK_DECL	unsigned int		Rec_PointCloud_getHeight(Rec_PointCloudPtr theCloud);

	/**Random access to a cloud's points and validity flags. Fills "xyz" with the cartesian Local coordinates of point with index "pointIdx".*/
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_getPoint(Rec_PointCloudPtr theCloud, unsigned int pointIdx, float xyz[3]);

	/**Same as above, only for structured point clouds, retrieves local coordinates of point at coords in structure (row, col)*/
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_getPointFromRowCol(Rec_PointCloudPtr theCloud, unsigned int row, unsigned int col, float xyz[3]);

	/**Retrieves flags of point at given index.*/
	REC3SDK_DECL	Rec_PointFlags		Rec_PointCloud_getPointFlags(Rec_PointCloudPtr theCloud, unsigned int pointIdx);

	/** Retrieves point flags of structured cloud at given row and column. Raises an REC_CALL_INVALID_FOR_CLOUD_NOT_STRUCTURED error if the cloud is not structured. */
	REC3SDK_DECL	Rec_PointFlags		Rec_PointCloud_getPointFlagsFromRowCol(Rec_PointCloudPtr theCloud, unsigned int row, unsigned int col);

	/** returns whether the i-th point in theCloud is drawable. For the definition of drawable point, check the declaration of REC_FLAGS_POINT_NOT_DRAWABLE. */
	REC3SDK_DECL	bool				Rec_PointCloud_isPointDrawable(Rec_PointCloudPtr theCloud, unsigned int pointIdx);

	/**Returns a pointer the cloud's raw buffer of points. This buffer is composed of 3 * NumPoints floats, and sequentially stores the x, y, and z local coordinates of each point.
	 * Memory ownership: the SDK deallocates this buffer when it is time, so the client program should not do that. This buffer can be also reallocated when the cloud is resized,
	 * invalidating the returned pointer. 
	 */
	REC3SDK_DECL	float*				Rec_PointCloud_getPointsRawBuffer(Rec_PointCloudPtr theCloud);

	/**Returns a pointer to the cloud's row buffer of point flags. The buffer is composed of NumPoints "Rec_PointFlags".
	 * Memory ownership: the SDK deallocates this buffer when it is time, so the client program should not do that. This buffer can be also reallocated when the cloud is resized.
	 */
	REC3SDK_DECL	Rec_PointFlags*		Rec_PointCloud_getPointFlagsRawBuffer(Rec_PointCloudPtr theCloud);

	/** Returns the global pose matrix of the point cloud, expressed as a row-major array of values.*/
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_getGlobalPoseMatrix(Rec_PointCloudPtr theCloud, double gpm[4 * 4]);

	/**Returns the point cloud's bounding box, expressed as two opposite vertices of the box in local coordinates.*/
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_getBoundingBox(Rec_PointCloudPtr theCloud, double bbmin[3], double bbmax[3]);

	/** Returns a string describing the laser scanner used to acquire the point cloud. */
	REC3SDK_DECL	const char*			Rec_PointCloud_getSensorUsed(Rec_PointCloudPtr theCloud);

	/**READING COLOR LAYERS' DATA AND PROPERTIES*/

	/** Returns the color layer's name. Memory ownership: the returned pointer is owned by the SDK and therefore cannot be deleted by the client program.*/
	REC3SDK_DECL	const char*			Rec_ColorLayer_getName(Rec_ColorLayerPtr theColor);

	/** Returns the color layer's data type. 
	 * The string "dataType" is 2 characters long. The first character is a digit to indicate how many fields per point the color layer stores.
	 * The second characters is one of 'b', 's', 'i', 'f', 'd', to indicate respectively that the field type is unsigned char, unsigned short, unsigned int, float, or double.
	 * Common examples: dataType is "1f" for reflectance and confidence, "3f" for point normals.
	 * Memory ownership: the returned pointer is owned by the SDK and therefore cannot be deleted by the client program.*/
	REC3SDK_DECL	const char*			Rec_ColorLayer_getDataType(Rec_ColorLayerPtr theColor);

	/**Returns the number of data fields for each point in the current color layer. This is 1 for "1f", 3 for "3f", etc.*/
	REC3SDK_DECL	unsigned short		Rec_ColorLayer_getNumFields(Rec_ColorLayerPtr theColor);

	/**Returns the number of bytes for each point in the current color layer. This is 4 for "1f", 3 for "3b", 12 for "3f", etc.*/
	REC3SDK_DECL	unsigned short		Rec_ColorLayer_getDataFieldSize(Rec_ColorLayerPtr theColor);

	/**Returns a pointer to the i-th record in the color data channel.*/
	REC3SDK_DECL	unsigned char*		Rec_ColorLayer_getDataField(Rec_ColorLayerPtr theColor, const unsigned int i);

	/**Returns a pointer to the record at index (row, col) in the color data channel of a structured point cloud.*/
	REC3SDK_DECL	unsigned char*		Rec_ColorLayer_getDataFieldFromRowCol(Rec_ColorLayerPtr theColor, unsigned int row, unsigned int col);

	/**Returns a memory buffer of size (number_of_points_in_the_cloud * data_field_size). */
	REC3SDK_DECL	unsigned char*		Rec_ColorLayer_getDataRawBuffer(Rec_ColorLayerPtr theColor);

	/**Returns a pointer to the i-th float record in the color data channel. 
	 * If the type of the color data is not float, a REC_COLOR_LAYER_DYNAMIC_TYPECHECK_FAILED error is raised and null pointer is returned. 
	 * Therefore, the data type of the color layer must be "1f"*/
	REC3SDK_DECL	float*				Rec_ColorLayer_getFloatDataField(Rec_ColorLayerPtr theColor, const unsigned int i);

	/**WRITING FUNCTIONS*/
	/** Creates an empty unstructured point cloud, reserves in it enough memory to store "numPoints" points, and returns a pointer to the cloud (memory is owned by the SDK and must not be deallocated).
	 * Points, point flags and colors can be inserted in the newly created cloud. However, to save it, the client program must tell the SDK where to save it, by calling the function 
	 * Rec_PointCloud_setFilepath() and then the function Rec_PointCloud_save().
	 */
	REC3SDK_DECL	Rec_PointCloudPtr	Rec_createEmptyUnstructuredCloud(const unsigned int numPoints);

	/** Creates an empty structred point cloud, reserving in it enough space to store "witth * height" points. Same comments as above. */
	REC3SDK_DECL	Rec_PointCloudPtr	Rec_createEmptyStructuredCloud(const unsigned int width, const unsigned int height);

	/**
	 * Adds a new color layer to "theCloud", initializing it with the given name and dataType.
	 * The string "dataType" must be 2 characters long. The first character should be a digit to indicate how many fields per point the color layer stores.
	 * The second characters must be one of 'b', 's', 'i', 'f', 'd', to indicate respectively that the field type is unsigned char, unsigned short, unsigned int, float, or double.
	 * Common examples: dataType is "1f" for reflectance and confidence, "3f" for point normals.
	 * Memory ownership: the SDK deallocates the returned pointer, on prorgam exit or when closing the point cloud,
	 * therfore do NOT delete it otherwise the program crashes.
	 */
	REC3SDK_DECL	Rec_ColorLayerPtr	Rec_PointCloud_addColorLayer(Rec_PointCloudPtr theCloud, const char* colorName, const char* dataType);

	/** Sets the global pose matrix of the point cloud, expressed as a row-major array of values.*/
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_setGlobalPoseMatrix(Rec_PointCloudPtr theCloud, const double gpm[4 * 4]);

	/** Sets the absolute path of the file where to save the point cloud. */
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_setFilepath(Rec_PointCloudPtr theCloud, const wchar_t* filePath);

	/** Saves the given point cloud to the given filepath. */
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_save(Rec_PointCloudPtr theCloud);

	/** Sets the cartesian coordinates of the point at index "pointIdx" in the give cloud. */
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_setPoint(Rec_PointCloudPtr theCloud, unsigned int pointIdx, const float xyz[3]);				

	/** Sets the cartesian coordinates of the point at given row and column in the given structured cloud. If the cloud is not structured, or the row and column are out of range, or
	 * the cloud pointer is invalid, corresponding errors are raised.
	 */
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_setPointFromRowCol(Rec_PointCloudPtr theCloud, unsigned int row, unsigned int col, const float xyz[3]);

	/** Sets the point flags for the point at index "pointIdx" in the given cloud. */
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_setPointFlags(Rec_PointCloudPtr theCloud, unsigned int pointIdx, Rec_PointFlags flags);

	/** Sets the point flags for the point at row, col in the given structured cloud. */
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_setPointFlagsFromRowCol(Rec_PointCloudPtr theCloud, unsigned int row, unsigned int col, Rec_PointFlags flags);

	/**Unloads and deallocates all data and resources related to the point cloud identified by "theCloud". If data are not saved, they will be lost.
	 * After calling this function, the pointer "theCloud" is not valid anymore, and calling SDK functions with this pointer as argument will raise an REC_INVALID_POINT_CLOUD_POINTER error.
	 */
	REC3SDK_DECL	Rec_ErrorCode		Rec_PointCloud_close(Rec_PointCloudPtr theCloud);

	/**This function release all RAM memory allocated by the SDK and its data structures. It can be called by the client program to make sure that no point clouds are occupying RAM memory
	 * after that all jobs are finished.
	 */
	REC3SDK_DECL	Rec_ErrorCode		Rec_releaseAllSDKresources();

#ifdef __cplusplus
}
#endif

#endif

