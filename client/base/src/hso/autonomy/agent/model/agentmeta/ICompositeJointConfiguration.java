/* Copyright 2008 - 2017 Hochschule Offenburg
 * For a list of authors see README.md
 * This software of HSOAutonomy is released under MIT License (see LICENSE).
 */
package hso.autonomy.agent.model.agentmeta;

/**
 * Universal joint configuration interface
 *
 * @author Stefan Glaser
 */
public interface ICompositeJointConfiguration extends ISensorConfiguration {
	IHingeJointConfiguration[] getHjConfigurations();

	/**
	 * @return a Mapper that maps joint angles to motor angles and vice versa.
	 *         null if no mapping is required.
	 */
	IJointToMotorMapper getJointToMotorMapper();
}
