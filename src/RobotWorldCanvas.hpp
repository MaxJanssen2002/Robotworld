#ifndef ROBOTWORLDCANVAS_HPP_
#define ROBOTWORLDCANVAS_HPP_

#include "Config.hpp"

#include "NotificationHandler.hpp"
#include "RobotWorld.hpp"
#include "Shape.hpp"
#include "ViewObject.hpp"
#include "Widgets.hpp"

#include <vector>

namespace View
{

	class Shape;
	typedef std::shared_ptr<Shape> ShapePtr;
	class RobotShape;
	typedef std::shared_ptr<RobotShape> RobotShapePtr;
	class GoalShape;
	typedef std::shared_ptr<GoalShape> GoalShapePtr;
	class WayPointShape;
	typedef std::shared_ptr<WayPointShape> WayPointShapePtr;
	class WallShape;
	typedef std::shared_ptr<WallShape> WallShapePtr;

	/**
	 *
	 */
	class RobotWorldCanvas :	public wxScrolledCanvas,
								public ViewObject
	{
		public:
			/**
			 *
			 */
			explicit RobotWorldCanvas( wxWindow* anOwner);
			/**
			 *
			 */
			RobotWorldCanvas( wxWindow* anOwner,
							  Model::ModelObjectPtr aModelObject);
			/**
			 *
			 */
			virtual ~RobotWorldCanvas();
			/**
			 * The canvas is partially displayed in a scrolling window. Hence, the user viewable part of this window
			 * is probably just a part of this window. This function takes a point in the (visible) scrolling window coordinates
			 * and translates it to this canvas windows coordinates. It takes into account the number of lines and number of
			 * pixels between lines.
			 */
			wxPoint devicePointFor( const wxPoint& aScreenPoint) const;
			/**
			 * This function is the opposite of devicePointFor(): it translates canvas coordinates to scrolling window coordinates
			 *
			 * @see devicePointFor( const wxPoint&)
			 */
			wxPoint screenPointFor( const wxPoint& aDevicePoint) const;
			/**
			 * @name Event handling enabling functions
			 *
			 */
			//@{
			void enableHandlePaint( bool enable = true);
			void enableHandleSize( bool enable = true);

			void enableLeftDownHandling( bool enable = true);
			void enableLeftUpHandling( bool enable = true);
			void enableLeftDClickHandling( bool enable = true);

			void enableMiddleDownHandling( bool enable = true);
			void enableMiddleUpHandling( bool enable = true);
			void enableMiddleDClickHandling( bool enable = true);

			void enableRightDownHandling( bool enable = true);
			void enableRightUpHandling( bool enable = true);
			void enableRightDClickHandling( bool enable = true);

			void enableMouseMotionHandling( bool enable = true);

			void enableKeyHandling( bool enable = true);

			void enableActivationHandling( bool enable = true);
			void enableSelectionHandling( bool enable = true);

			void enableItemMenuHandling( bool enable = true);

			void enableDragAndDropHandling( bool enable = true);
			//@}
			/**
			 *
			 * @return true if any shape is selected, false otherwise
			 */
			bool isShapeSelected() const;
			/**
			 *
			 * @return A selected Shape if any, nullptr otherwise.
			 */
			virtual ShapePtr getSelectedShape() const;
			/**
			 * Sets the given Shape as the selected shape.
			 * If selectionEnabled is set to true, handleSelection() of both the old and new
			 * selected shape will be called after setting their selection as appropriate.
			 *
			 * @param 	aSelectedShape 	The new Shape that will be selected. If a nullptr is passed  it acts as a mere de-selection of the current
			 * 							selected shape
			 */
			virtual void setSelectedShape( ShapePtr aSelectedShape);

			/**
			 *
			 * @param 	aPoint
			 * @return 	True if any Shape returns true for Shape.ocuppies(aPoint), false otherwise.
			 *
			 * @see Shape::occupies(const wxPoint&)
			 */
			virtual bool isShapeAt( const wxPoint& aPoint) const;
			/**
			 *
			 * @param 	aPoint A screen point, i.e. a point on the screen, not on the (scrollable) canvas.
			 * @return 	The first Shape in iteration order that returns true for Shape.ocuppies(aPoint). If
			 * 			no such Shape exists nullptr will be returned.
			 */
			virtual ShapePtr getShapeAt( const wxPoint& aPoint) const;
			/**
			 * Selects the Shape that returns true for Shape.ocuppies(aPoint).
			 * @param aPoint
			 * @return
			 */
			virtual bool selectShapeAt( const wxPoint& aPoint);

			/**
			 * @name Type safe accessors and mutators
			 */
			//@{
			/**
			 * Type safe accessor
			 */
			Model::RobotWorldPtr getRobotWorld() const;
			/**
			 * Type safe mutator
			 */
			void setRobotWorld( Model::RobotWorldPtr aRobotWorld);
			//@}
			/**
			 * @name Observer functions
			 */
			//@{
			/**
			 * A Notifier will call this function if this Observer will handle the notifications of that
			 * Notifier. It is the responsibility of the Observer to filter any events it is interested in.
			 * This function should only be called from the main thread because wxWidgets does not allow
			 * for painting widgets in a background thread: all painting should be done in the main thread.
			 * Therefore all notifications are routed to handleBackGroundNotification() as a convenience.
			 *
			 */
			virtual void handleNotification() override;
			//@}
			/**
			 * A Notifier that runs in a background thread should call this function instead of handleNotification().
			 * handleNotification() is routed to this function as a convenience. Bad for performance though.
			 */
			virtual void handleBackGroundNotification();
			/**
			 * Asks the world to populates itself with a robot, a goal and the given number of walls
			 */
			void populate( int aNumberOfWalls = 2);
			/**
			 * Removes everything from the world
			 */
			void unpopulate();
			/**
			 * Asks the robotworld to put the walls and goals in the world to represent a certain scenario
			 * @param scenarioNumber The index of the scenario
			 */
			void createScenario(Model::Scenarios scenarioNumber);
		protected:
			/**
			 * Common initialise function
			 */
			void initialise();
			/**
			 *
			 */
			void render( wxDC& dc);
			/**
			 * @name Event handling functions
			 *
			 * Override these functions if the default handling is not what you want
			 */
			//@{
			virtual void handlePaint( wxPaintEvent& event);
			virtual void handleSize( wxSizeEvent& event);

			virtual void handleLeftDown( wxMouseEvent& event);
			virtual void handleLeftUp( wxMouseEvent& event);
			virtual void handleLeftDClick( wxMouseEvent& event);

			virtual void handleMiddleDown( wxMouseEvent& event);
			virtual void handleMiddleUp( wxMouseEvent& event);
			virtual void handleMiddleDClick( wxMouseEvent& event);

			virtual void handleRightDown( wxMouseEvent& event);
			virtual void handleRightUp( wxMouseEvent& event);
			virtual void handleRightDClick( wxMouseEvent& event);

			virtual void handleMouseMotion( wxMouseEvent& event);

			virtual void handleKey( wxKeyEvent& event);

			virtual void handleBeginLeftDrag( ShapePtr aShape);
			virtual void handleEndDrag( ShapePtr aShape);

			virtual void handleAddRobot( wxCommandEvent& event);
			virtual void handleEditRobot( wxCommandEvent& event);
			virtual void handleDeleteRobot( wxCommandEvent& event);

			virtual void handleAddWayPoint( wxCommandEvent& event);
			virtual void handleEditWayPoint( wxCommandEvent& event);
			virtual void handleDeleteWayPoint( wxCommandEvent& event);

			virtual void handleAddGoal( wxCommandEvent& event);
			virtual void handleEditGoal( wxCommandEvent& event);
			virtual void handleDeleteGoal( wxCommandEvent& event);

			virtual void handleAddWall( wxCommandEvent& event);
			virtual void handleEditWall( wxCommandEvent& event);
			virtual void handleDeleteWall( wxCommandEvent& event);

			virtual void handleShapeInfo( wxCommandEvent& event);

			virtual void handleNotification( wxNotifyEvent& aNotifyEvent);

			//@}
			virtual void handleActivation( ShapePtr aShape);
			virtual void handleSelection( ShapePtr aShape);
			/**
			 *
			 */
			virtual void handleMenu( const wxPoint& aScreenPoint);
			/**
			 *
			 */
			virtual void handleItemMenu( 	ShapePtr aSelectedShape,
											const wxPoint& aPoint);
			/**
			 *
			 */
			void addShape( RobotShapePtr aRobotShape);
			/**
			 *
			 */
			void addShape( GoalShapePtr aGoalShape);
			/**
			 *
			 */
			void addShape( WayPointShapePtr aWayPointShape);
			/**
			 *
			 */
			void addShape( WallShapePtr aWallShape);
			/**
			 *
			 */
			void removeShape( RobotShapePtr aRobotShape);
			/**
			 *
			 */
			void removeShape( GoalShapePtr aGoalShape);
			/**
			 *
			 */
			void removeShape( WayPointShapePtr aWayPointShape);
			/**
			 *
			 */
			void removeShape( WallShapePtr aWallShape);
			/**
			 *
			 */
			void removeGenericShape( ShapePtr aShape);
		private:
			/**
			 * @name Event handlers
			 *
			 */
			//@{
			void OnPaint( wxPaintEvent& event);
			void OnSize( wxSizeEvent& event);

			void OnLeftDown( wxMouseEvent& event);
			void OnLeftUp( wxMouseEvent& event);
			void OnLeftDClick( wxMouseEvent& event);

			void OnMiddleDown( wxMouseEvent& event);
			void OnMiddleUp( wxMouseEvent& event);
			void OnMiddleDClick( wxMouseEvent& event);

			void OnRightDown( wxMouseEvent& event);
			void OnRightUp( wxMouseEvent& event);
			void OnRightDClick( wxMouseEvent& event);

			void OnMouseMotion( wxMouseEvent& event);

			void OnKeyDown( wxKeyEvent& event);
			void OnCharDown( wxKeyEvent& event);

			void OnAddRobot( wxCommandEvent& event);
			void OnEditRobot( wxCommandEvent& event);
			void OnDeleteRobot( wxCommandEvent& event);

			void OnAddWayPoint( wxCommandEvent& event);
			void OnEditWayPoint( wxCommandEvent& event);
			void OnDeleteWayPoint( wxCommandEvent& event);

			void OnAddGoal( wxCommandEvent& event);
			void OnEditGoal( wxCommandEvent& event);
			void OnDeleteGoal( wxCommandEvent& event);

			void OnAddWall( wxCommandEvent& event);
			void OnEditWall( wxCommandEvent& event);
			void OnDeleteWall( wxCommandEvent& event);

			void OnShapeInfo( wxCommandEvent& event);

			void OnWorldInfo( wxCommandEvent& event);

			void OnGenerateWorldCode( wxCommandEvent& event);

			void OnNotificationEvent( wxNotifyEvent& aNotifyEvent);
			//@}

			std::vector< ShapePtr > shapes;

			enum
			{
				IDLE,
				DRAWING,
				CANCELDRAWING,
				RESIZING,
				CANCELRESIZING,
				STARTDRAGGING,
				DRAGGING,
				CANCELDRAGGING
			} actionStatus;

			wxPoint popupPoint;
			wxPoint startActionPoint;
			wxPoint endActionPoint;
			wxPoint actionOffset;
			wxSize startActionSize;

			ShapePtr startActionShape;
			ShapePtr endActionShape;

			ShapePtr selectedShape;

			bool activationEnabled;
			bool selectionEnabled;
			bool menuItemEnabled;
			bool dandEnabled;

			Base::NotificationHandler< std::function< void( wxNotifyEvent&) > > * notificationHandler;

			/**
			 * This function removes all Shapes that look at a ModelObject that is not longer in RobotWorld
			 */
			template< typename T, typename S >
			void remove( const std::vector<std::shared_ptr< T > >& aTs)
			{
				std::vector< ShapePtr> shapePtrs;
				std::copy_if(	shapes.begin(),
								shapes.end(),
								std::back_inserter(shapePtrs),
								[&aTs](ShapePtr aShape)
								{
									std::shared_ptr< S > shape = std::dynamic_pointer_cast< S >( aShape);
									if (shape)
									{
										return std::find_if(	aTs.begin(),
																aTs.end(),
																[shape](std::shared_ptr< T > aT)
																{
																	return shape->getModelObject()->getObjectId() == aT->getObjectId();
																}) == aTs.end();
									}
									return false;
								});
				for(ShapePtr shape : shapePtrs)
				{
					removeShape( std::dynamic_pointer_cast< S >( shape));
				}
			}
			/**
			 * This function add Shapes for ModelObjects that are in RobotWorld but that have no Shape yet
			 */
			template< typename T, typename S >
			void add( const std::vector<std::shared_ptr< T > >& aTs)
			{
				for(std::shared_ptr< T > t : aTs)
				{
					auto result = std::find_if(	shapes.begin(),
												shapes.end(),
												[t](ShapePtr aShape)
												{
													if(aShape->getModelObject())
													{
														return aShape->getModelObject()->getObjectId() == t->getObjectId();
													}
													return false;
												});
					if( result == shapes.end())
					{
						addShape(std::shared_ptr< S >(new S(t)));
					}
				}
			}
	};
} // namespace View
#endif /* ROBOTWORLDCANVAS_HPP_ */
